cmake_minimum_required(VERSION 2.8.3)
project(web_teleop)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  fetch_api
)

catkin_python_setup()

add_service_files(
  FILES
  SetTorso.srv
  CloseGripper.srv
  OpenGripper.srv
  SetArm.srv
  MoveHead.srv
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp rospy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


# unit tests are enabled selectively
if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch/app.launch)
endif()
