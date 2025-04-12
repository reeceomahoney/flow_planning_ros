cmake_minimum_required(VERSION 3.0.2)
project(flow_planning)

find_package(catkin REQUIRED COMPONENTS rospy)

catkin_package()

include_directories( ${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS
  src/controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
