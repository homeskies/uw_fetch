#!/usr/bin/env python
import sys
from knowledge_representation import PyAttributeList, AttributeValueType, Entity, Concept, Instance
import knowledge_representation


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class Database():
    def __init__(self):
        self.ltmc = knowledge_representation.get_default_ltmc()

    def add_pose(self):
        nsb_concept = self.ltmc.get_concept("never seen before")
        instance = nsb_concept.create_instance()
        result_list = PyAttributeList()
        self.ltmc.select_query_string("SELECT * FROM entity_attributes_str", result_list)
        print "TEST:"
        for r in result_list:
            print str(r)


def main():
    # rospy.init_node('server_svg')
    # wait_for_time()
    
    database = Database()
    database.add_pose()

    # server = Server(database)
    # save_sub = rospy.Subscriber("map_annotator/user_actions", UserAction, server.handleAction)


    # def handle_shutdown():
        # database.save()

    # rospy.on_shutdown(handle_shutdown)
    # rospy.spin()


if __name__ == '__main__':
    main()