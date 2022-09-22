# Here we find a typical ROS description package, this set the KORTEX_DESCRIPTION_PATH CMake variable
find_description_package(kortex_description)
# Generate the robot module C++ code using this value
configure_file(KinovaRobotModule.in.cpp "${CMAKE_CURRENT_BINARY_DIR}/KinovaRobotModule.cpp")

set(module_SRC
  "${CMAKE_CURRENT_BINARY_DIR}/KinovaRobotModule.cpp"
)

set(module_HDR
  KinovaRobotModule.h
)

add_robot(kinova "${module_SRC}" "${module_HDR}")
target_include_directories(kinova PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
