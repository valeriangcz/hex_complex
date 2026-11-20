cmake_minimum_required(VERSION 3.0.2)
project(hex_expert)

find_package(catkin REQUIRED COMPONENTS hex_kinematic map_robot_voxles)
catkin_python_setup()
catkin_package(INCLUDE_DIRS include LIBRARIES hex_expert_cpp CATKIN_DEPENDS hex_kinematic map_robot_voxles)
add_library(hex_expert_cpp src/expert_climb.cpp)
target_compile_features(hex_expert_cpp PUBLIC cxx_std_17)
target_include_directories(hex_expert_cpp PUBLIC include ${catkin_INCLUDE_DIRS})
target_compile_definitions(hex_expert_cpp PUBLIC EIGEN_DONT_PARALLELIZE)
target_link_libraries(hex_expert_cpp PUBLIC hex_kinematic_cpp)
install(TARGETS hex_expert_cpp ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

catkin_install_python(PROGRAMS scripts/swing_traj_vis.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
