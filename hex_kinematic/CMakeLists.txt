cmake_minimum_required(VERSION 3.0.2)
project(hex_kinematic)

find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(TINYXML2 REQUIRED tinyxml2)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
catkin_python_setup()
catkin_package(INCLUDE_DIRS include LIBRARIES hex_kinematic_cpp DEPENDS Eigen3 TINYXML2)

add_library(hex_kinematic_cpp src/kinematic.cpp)
target_compile_features(hex_kinematic_cpp PUBLIC cxx_std_17)
target_compile_definitions(hex_kinematic_cpp PUBLIC EIGEN_DONT_PARALLELIZE)
target_include_directories(hex_kinematic_cpp PUBLIC include ${EIGEN3_INCLUDE_DIRS} ${TINYXML2_INCLUDE_DIRS})
target_link_libraries(hex_kinematic_cpp PUBLIC Eigen3::Eigen ${TINYXML2_LIBRARIES})
install(TARGETS hex_kinematic_cpp ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY resources
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
