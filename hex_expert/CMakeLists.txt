cmake_minimum_required(VERSION 3.0.2)
project(hex_expert)

find_package(catkin REQUIRED COMPONENTS hex_kinematic map_robot_voxles)
catkin_python_setup()
catkin_package(CATKIN_DEPENDS hex_kinematic map_robot_voxles)

catkin_install_python(PROGRAMS scripts/swing_traj_vis.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
