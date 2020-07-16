import copy
import actionlib
import rospy

from math import sin, cos
from moveit_python import (MoveGroupInterface,
                           PlanningSceneInterface,
                           PickPlaceInterface)
from moveit_python.geometry import rotate_pose_msg_by_euler_angles
from grasping_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal
# from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes

# Tools for grasping
class GraspingClient(object):

    def __init__(self):
        rospy.loginfo("Grasping Client for demo purposes")
        rospy.loginfo("If your script hangs here it is likely you did not runt the task_storing_groceries script")
        self.scene = PlanningSceneInterface("base_link")
        self.pickplace = PickPlaceInterface("arm", "gripper", verbose=True)
        self.move_group = MoveGroupInterface("arm", "base_link")

        find_topic = "basic_grasping_perception/find_objects"
        rospy.loginfo("Waiting for %s..." % find_topic)
        self.find_client = actionlib.SimpleActionClient(find_topic, FindGraspableObjectsAction)
        self.find_client.wait_for_server()

    def updateScene(self):
        # find objects
        goal = FindGraspableObjectsGoal()
        goal.plan_grasps = True
        self.find_client.send_goal(goal)
        self.find_client.wait_for_result(rospy.Duration(5.0))
        find_result = self.find_client.get_result()

        # remove previous objects
        for name in self.scene.getKnownCollisionObjects():
            self.scene.removeCollisionObject(name, False)
        for name in self.scene.getKnownAttachedObjects():
            self.scene.removeAttachedObject(name, False)
        self.scene.waitForSync()

        # insert objects to scene
        idx = -1
        for obj in find_result.objects:
            idx += 1
            obj.object.name = "object%d"%idx
            self.scene.addSolidPrimitive(obj.object.name,
                                         obj.object.primitives[0],
                                         obj.object.primitive_poses[0],
                                         False)

        for obj in find_result.support_surfaces:
            # extend surface to floor, and make wider since we have narrow field of view
            height = obj.primitive_poses[0].position.z
            obj.primitives[0].dimensions = [obj.primitives[0].dimensions[0],
                                            1.5,  # wider
                                            obj.primitives[0].dimensions[2] + height]
            obj.primitive_poses[0].position.z += -height/2.0

            # add to scene
            self.scene.addSolidPrimitive(obj.name,
                                         obj.primitives[0],
                                         obj.primitive_poses[0],
                                         False)

        self.scene.waitForSync()

        # store for grasping
        self.objects = find_result.objects
        self.surfaces = find_result.support_surfaces

    def getGraspableCube(self):
        graspable = None
        for obj in self.objects:
            # self.getCokeReference(obj)
            # need grasps
            if len(obj.grasps) < 1:
                continue
            # check size
            print("dims")
            for i in obj.object.primitives[0].dimensions:
                print(i)
            # print(obj.object.primitives[0].dimensions[0])
            # print(obj.object.primitives[0].dimensions[1])
            # print(obj.object.primitives[0].dimensions[2])
            if obj.object.primitives[0].dimensions[0] < 0.02 or \
               obj.object.primitives[0].dimensions[0] > 0.14 or \
               obj.object.primitives[0].dimensions[0] < 0.02 or \
               obj.object.primitives[0].dimensions[0] > 0.14 or \
               obj.object.primitives[0].dimensions[0] < 0.02 or \
               obj.object.primitives[0].dimensions[0] > 0.14:
                continue
            # has to be on table
            print("z")
            print(obj.object.primitive_poses[0].position.z)
            if obj.object.primitive_poses[0].position.z < 0.2:
                continue
            return obj.object, obj.grasps
        # nothing detected
        return None, None

    # def getCokeReference(self, obj):
    #     pose = obj.object.primitive_poses[0]
    #     pose_stamped = PoseStamped()
    #     pose_stamped.pose = pose
    #     transform = tf_buffer.lookup_transform("map",
    #                                    "head_camera_link", #source frame
    #                                    rospy.Time(0), #get the tf at first available time
    #                                    rospy.Duration(1.0)) #wait for 1 second
    #     pose_transformed = do_transform_pose(pose_stamped, transform)
    #     print pose_transformed
    def getSupportSurface(self, name):
        for surface in self.support_surfaces:
            if surface.name == name:
                return surface
        return None

    def getPlaceLocation(self):
        pass

    def pick(self, block, grasps):
        success, pick_result = self.pickplace.pick_with_retry(block.name,
                                                              grasps,
                                                              support_name=block.support_surface,
                                                              scene=self.scene)
        self.pick_result = pick_result
        return success

    def place(self, block, pose_stamped):
        places = list()
        l = PlaceLocation()
        l.place_pose.pose = pose_stamped.pose
        l.place_pose.header.frame_id = pose_stamped.header.frame_id

        # copy the posture, approach and retreat from the grasp used
        l.post_place_posture = self.pick_result.grasp.pre_grasp_posture
        l.pre_place_approach = self.pick_result.grasp.pre_grasp_approach
        l.post_place_retreat = self.pick_result.grasp.post_grasp_retreat
        places.append(copy.deepcopy(l))
        # create another several places, rotate each by 360/m degrees in yaw direction
        m = 16 # number of possible place poses
        pi = 3.141592653589
        for i in range(0, m-1):
            l.place_pose.pose = rotate_pose_msg_by_euler_angles(l.place_pose.pose, 0, 0, 2 * pi / m)
            places.append(copy.deepcopy(l))

        success, place_result = self.pickplace.place_with_retry(block.name,
                                                                places,
                                                                scene=self.scene)
        return success

    def tuck(self):
        joints = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        pose = [1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]
        while not rospy.is_shutdown():
            result = self.move_group.moveToJointPosition(joints, pose, 0.02)
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return