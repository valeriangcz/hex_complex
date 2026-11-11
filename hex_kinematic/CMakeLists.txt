cmake_minimum_required(VERSION 3.0.2)
project(hex_kinematic)

find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()

install(DIRECTORY resources
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
