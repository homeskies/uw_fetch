cmake_minimum_required(VERSION 2.8.3)
find_package(catkin)
project(astro_xavier_bringup)
catkin_package()

install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
