cmake_minimum_required(VERSION 3.0.2)
project(hex_isaacgym)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  geometry_msgs
  hex_expert
  hex_kinematic
  hex_se3
  nav_msgs
  rosgraph_msgs
  rospy
  sensor_msgs
  std_srvs
)
catkin_python_setup()
catkin_package(CATKIN_DEPENDS
  diagnostic_msgs
  geometry_msgs
  hex_expert
  hex_kinematic
  hex_se3
  nav_msgs
  rosgraph_msgs
  rospy
  sensor_msgs
  std_srvs
)

catkin_install_python(PROGRAMS
  scripts/expert_sim_node.py
  scripts/test_backend.py
  scripts/test_closed_loop.py
  scripts/test_climb.py
  scripts/test_climb_ros.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
