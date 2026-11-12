cmake_minimum_required(VERSION 3.0.2)
project(hex_runner)
find_package(catkin REQUIRED COMPONENTS diagnostic_msgs geometry_msgs nav_msgs rospy sensor_msgs std_srvs dm_motor_fusion feetech_servo magnetic hex_kinematic hex_expert hex_se3)
catkin_python_setup()
catkin_package(CATKIN_DEPENDS diagnostic_msgs geometry_msgs nav_msgs rospy sensor_msgs std_srvs dm_motor_fusion feetech_servo magnetic hex_kinematic hex_expert hex_se3)
catkin_install_python(PROGRAMS scripts/runner_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(tests)
endif()
