cmake_minimum_required(VERSION 3.0.2)
project(feetech_servo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)
add_message_files(
  FILES
  servo_cmd.msg
  servo_state.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS src
  LIBRARIES SCServo
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)

add_subdirectory(src)

add_executable(feetech_servo_interface interface.cpp)
target_compile_options(feetech_servo_interface PRIVATE -std=c++11)
target_include_directories(feetech_servo_interface PRIVATE ${catkin_INCLUDE_DIRS})
add_dependencies(feetech_servo_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(feetech_servo_interface SCServo ${catkin_LIBRARIES})

install(TARGETS feetech_servo_interface
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch para
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS SCServo
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Keep the vendor's public include style: #include <SCServo.h>.
install(DIRECTORY src/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
