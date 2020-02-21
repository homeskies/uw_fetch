cmake_minimum_required(VERSION 3.0.2)
project(uw_tools)
find_package(catkin REQUIRED)
catkin_package()

# TODO: Detect whether we're installing on the robot here?
install(FILES scripts/uw_tools_client
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_add_env_hooks(${PROJECT_NAME} SHELLS bash zsh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)