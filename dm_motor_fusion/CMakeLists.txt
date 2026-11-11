cmake_minimum_required(VERSION 3.0.2)
project(dm_motor_fusion)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(${PROJECT_NAME}_node
  src/dm_motor_fusion_node.cpp
  src/hardware_interface/socketcan.cpp
  src/hardware_interface/damiao.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  pthread
)

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
