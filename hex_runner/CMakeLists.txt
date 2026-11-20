cmake_minimum_required(VERSION 3.0.2)
project(hex_runner)
find_package(catkin REQUIRED COMPONENTS diagnostic_msgs geometry_msgs rospy roscpp roslib std_msgs std_srvs dm_motor_fusion feetech_servo magnetic hex_kinematic hex_expert hex_se3)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)
catkin_python_setup()
catkin_package(INCLUDE_DIRS include LIBRARIES hex_runner_core
  DEPENDS YAML_CPP JSONCPP
  CATKIN_DEPENDS diagnostic_msgs geometry_msgs rospy roscpp roslib std_msgs std_srvs dm_motor_fusion feetech_servo magnetic hex_kinematic hex_expert hex_se3)
add_library(hex_runner_core src/config.cpp src/initialization.cpp src/runner.cpp)
target_compile_features(hex_runner_core PUBLIC cxx_std_17)
target_include_directories(hex_runner_core PUBLIC include ${catkin_INCLUDE_DIRS} ${JSONCPP_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIRS})
target_compile_definitions(hex_runner_core PUBLIC EIGEN_DONT_PARALLELIZE)
target_link_libraries(hex_runner_core PUBLIC hex_expert_cpp ${YAML_CPP_LIBRARIES} ${JSONCPP_LIBRARIES} ${catkin_LIBRARIES})
add_executable(runner_node_cpp src/runner_node.cpp)
target_link_libraries(runner_node_cpp PRIVATE hex_runner_core)
add_dependencies(runner_node_cpp ${catkin_EXPORTED_TARGETS})
install(TARGETS hex_runner_core runner_node_cpp
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
catkin_install_python(PROGRAMS scripts/runner_node.py scripts/sim_launcher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  add_executable(hex_cpp_probe tests/cpp_probe.cpp)
  target_link_libraries(hex_cpp_probe PRIVATE hex_runner_core)
  catkin_add_gtest(test_hex_runner_core tests/runner_core_test.cpp)
  if(TARGET test_hex_runner_core)
    target_compile_definitions(test_hex_runner_core PRIVATE CPP_TEST_URDF="${CMAKE_CURRENT_SOURCE_DIR}/../hex_kinematic/resources/hex_magnetic/urdf/hex_all.urdf")
    target_link_libraries(test_hex_runner_core hex_runner_core)
  endif()
  catkin_add_nosetests(tests DEPENDENCIES hex_cpp_probe)
endif()
