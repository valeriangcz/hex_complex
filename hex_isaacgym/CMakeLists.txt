cmake_minimum_required(VERSION 3.0.2)
project(hex_isaacgym)
find_package(catkin REQUIRED COMPONENTS diagnostic_msgs geometry_msgs nav_msgs rospy std_msgs std_srvs dm_motor_fusion feetech_servo magnetic hex_kinematic rosgraph_msgs)
catkin_python_setup()
catkin_package(CATKIN_DEPENDS diagnostic_msgs geometry_msgs nav_msgs rospy std_msgs std_srvs dm_motor_fusion feetech_servo magnetic hex_kinematic rosgraph_msgs)
catkin_install_python(PROGRAMS scripts/sim_node.py scripts/test_backend.py scripts/test_reset.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(tests)
endif()
