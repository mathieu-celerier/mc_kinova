cmake_minimum_required(VERSION 3.5)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 11)

set(PROJECT_NAME mc_kinova)
set(PROJECT_DESCRIPTION "KinovaRobotModule")
set(PROJECT_URL "")

include(GNUInstallDirs)

project(${PROJECT_NAME} LANGUAGES CXX VERSION 1.0.0)
include(CTest)
enable_testing()

if(MSVC AND _VCPKG_INSTALLED_DIR)
  set(VCPKG_INSTALL_PREFIX "${_VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_PREFIX}")
  # Only debug actually exists but we make sure it's first in that case
  set(CMAKE_MSVCIDE_RUN_PATH "${VCPKG_INSTALL_PREFIX}/\$(Configuration)/bin;${VCPKG_INSTALL_PREFIX}/bin;${CMAKE_MSVCIDE_RUN_PATH}")
endif()

find_package(mc_rtc REQUIRED)

# Here we find a typical ROS description package, this set the KORTEX_DESCRIPTION_PATH CMake variable
find_description_package(kortex_description)

find_program(XACRO xacro REQUIRED)

# Find all xacro files in the kortex_description package to use as a dependency for the urdf generation
file(GLOB_RECURSE KINOVA_XACRO_FILES "${KORTEX_DESCRIPTION_PATH}/*.xacro")

# Generate the URDF
set(GENERATED_URDF "${CMAKE_CURRENT_BINARY_DIR}/urdf/gen3_7dof_novision_sim.urdf")
set(KINOVA_URDF_DIR "${CMAKE_INSTALL_FULL_DATADIR}/mc_kinova/urdf")
set(KINOVA_URDF_PATH "${KINOVA_URDF_DIR}/gen3_7dof_novision_sim.urdf")
add_custom_command(OUTPUT ${GENERATED_URDF} COMMAND ${XACRO} -o ${GENERATED_URDF} -i ${KORTEX_DESCRIPTION_PATH}/robots/gen3.xacro dof:=7 vision:=false sim:=true DEPENDS ${KINOVA_XACRO_FILES})
add_custom_target(generate-kinova-urdf DEPENDS ${GENERATED_URDF})

add_subdirectory(src)

install(FILES ${GENERATED_URDF} DESTINATION ${KINOVA_URDF_DIR})
install(DIRECTORY convex DESTINATION ${CMAKE_INSTALL_DATADIR}/mc_kinova)

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()
