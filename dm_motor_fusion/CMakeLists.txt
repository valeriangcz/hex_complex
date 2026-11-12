cmake_minimum_required(VERSION 3.0.2)
project(dm_motor_fusion)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  LegCommand.msg
  LegState.msg
  MultiLegCommand.msg
  MultiLegState.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(${PROJECT_NAME}_node
  node/dm_motor_fusion_node.cpp
  src/socketcan.cpp
  src/damiao.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  pthread
)

# 节点使用本包生成的消息头，需要等消息生成完成
add_dependencies(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  add_executable(dm_feedback_test test/test_feedback.cpp src/damiao.cpp src/socketcan.cpp)
  target_link_libraries(dm_feedback_test ${catkin_LIBRARIES} pthread)
  add_test(NAME dm_feedback_validity COMMAND dm_feedback_test)
endif()
