cmake_minimum_required(VERSION 3.0.2)
project(map_robot_voxles)

find_package(catkin REQUIRED COMPONENTS hex_kinematic)
catkin_python_setup()
catkin_package(CATKIN_DEPENDS hex_kinematic)

catkin_install_python(PROGRAMS
  scripts/adopt_existing_cache.py
  scripts/build_map.py
  scripts/build_robot_cache.py
  scripts/view_map_robot.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config resources
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
