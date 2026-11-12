cmake_minimum_required(VERSION 3.0.2)
project(SCServo)

add_library(SCServo STATIC
  SCS.cpp
  SCSerial.cpp
  SCSCL.cpp
  SMS_STS.cpp
  HLSCL.cpp
)

target_compile_options(SCServo PRIVATE -std=c++11)
target_include_directories(SCServo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>
)
# Allow this static library to be linked into shared libraries.
set_target_properties(SCServo PROPERTIES POSITION_INDEPENDENT_CODE ON)
