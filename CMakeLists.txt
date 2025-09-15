cmake_minimum_required(VERSION 3.14)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 17)

set(PROJECT_NAME mc_kinova)
set(PROJECT_DESCRIPTION "KinovaRobotModule")
set(PROJECT_URL "")

include(GNUInstallDirs)

project(${PROJECT_NAME} LANGUAGES CXX VERSION 1.0.0)
include(CTest)
enable_testing()

# Build option to enable Bota sensor
option(WITH_BOTA_SENSOR "Enable Bota (Rokubimini) sensor integration" OFF)

if(MSVC AND _VCPKG_INSTALLED_DIR)
  set(VCPKG_INSTALL_PREFIX "${_VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_PREFIX}")
  set(CMAKE_MSVCIDE_RUN_PATH "${VCPKG_INSTALL_PREFIX}/\$(Configuration)/bin;${VCPKG_INSTALL_PREFIX}/bin;${CMAKE_MSVCIDE_RUN_PATH}")
endif()

find_package(mc_rtc REQUIRED)

find_description_package(kortex_description)

# Find program xacro
find_program(XACRO xacro REQUIRED)

# Glob XACRO files for dependency tracking
file(GLOB_RECURSE KINOVA_XACRO_FILES "${KORTEX_DESCRIPTION_PATH}/*.xacro")

# Bota sensor integration (conditional)
if(WITH_BOTA_SENSOR)
  message(STATUS "Bota sensor support ENABLED")
  find_description_package(rokubimini_description)
  file(GLOB_RECURSE BOTA_XACRO_FILES "${BOTA_DESCRIPTION_PATH}/*.xacro")
else()
  message(STATUS "Bota sensor support DISABLED")
  set(BOTA_XACRO_FILES "")
endif()

# Directory for generated URDF
set(KINOVA_URDF_DIR "${CMAKE_INSTALL_FULL_DATADIR}/mc_kinova/urdf")
set(KINOVA_XACRO_PATH "${CMAKE_CURRENT_BINARY_DIR}/xacro/gen3.xacro")
configure_file(${CMAKE_SOURCE_DIR}/xacro/gen3.in.xacro ${KINOVA_XACRO_PATH})

# Common URDF generation (no Bota sensor)
set(GENERATED_URDF "${CMAKE_CURRENT_BINARY_DIR}/urdf/gen3_7dof_novision_sim.urdf")
set(KINOVA_URDF_PATH "${KINOVA_URDF_DIR}/gen3_7dof_novision_sim.urdf")
add_custom_command(
  OUTPUT ${GENERATED_URDF}
  COMMAND ${XACRO} -o ${GENERATED_URDF} -i ${KORTEX_DESCRIPTION_PATH}/robots/gen3.xacro dof:=7 vision:=false sim:=true
  DEPENDS ${KINOVA_XACRO_FILES}
)

set(GENERATED_URDF_CAMERA "${CMAKE_CURRENT_BINARY_DIR}/urdf/gen3_7dof_vision_sim.urdf")
set(KINOVA_URDF_PATH_CAMERA "${KINOVA_URDF_DIR}/gen3_7dof_vision_sim.urdf")
add_custom_command(
  OUTPUT ${GENERATED_URDF_CAMERA}
  COMMAND ${XACRO} -o ${GENERATED_URDF_CAMERA} -i ${KINOVA_XACRO_PATH} dof:=7 vision:=true bota:=false DS4:=false
  DEPENDS ${KINOVA_XACRO_FILES} ${BOTA_XACRO_FILES}
)

set(GENERATED_URDF_CAMERA_GRIPPER "${CMAKE_CURRENT_BINARY_DIR}/urdf/gen3_7dof_vision_sim_gripper.urdf")
set(KINOVA_URDF_PATH_CAMERA_GRIPPER "${KINOVA_URDF_DIR}/gen3_7dof_vision_sim_gripper.urdf")
add_custom_command(
  OUTPUT ${GENERATED_URDF_CAMERA_GRIPPER}
  COMMAND ${XACRO} -o ${GENERATED_URDF_CAMERA_GRIPPER} -i ${KINOVA_XACRO_PATH} dof:=7 vision:=true gripper:="robotiq_2f_85" bota:=false DS4:=false
  DEPENDS ${KINOVA_XACRO_FILES} ${BOTA_XACRO_FILES}
)

set(GENERATED_URDF_FILES ${GENERATED_URDF} ${GENERATED_URDF_CAMERA} ${GENERATED_URDF_CAMERA})


# Bota-specific URDF (only if WITH_BOTA_SENSOR)
if(WITH_BOTA_SENSOR)
  # Configure the composite xacro file (template-based)
  set(KINOVA_XACRO_PATH "${CMAKE_CURRENT_BINARY_DIR}/xacro/gen3_bota.xacro")
  configure_file(${CMAKE_SOURCE_DIR}/xacro/gen3_bota.in.xacro ${KINOVA_XACRO_PATH})

  set(GENERATED_URDF_BOTA "${CMAKE_CURRENT_BINARY_DIR}/urdf/gen3_7dof_novision_sim_bota.urdf")
  set(KINOVA_URDF_PATH_BOTA "${KINOVA_URDF_DIR}/gen3_7dof_novision_sim_bota.urdf")
  add_custom_command(
    OUTPUT ${GENERATED_URDF_BOTA}
    COMMAND ${XACRO} -o ${GENERATED_URDF_BOTA} -i ${KINOVA_XACRO_PATH} dof:=7 vision:=false DS4:=false
    DEPENDS ${KINOVA_XACRO_FILES} ${BOTA_XACRO_FILES}
  )

  set(GENERATED_URDF_BOTA_DS4 "${CMAKE_CURRENT_BINARY_DIR}/urdf/gen3_7dof_novision_sim_bota_ds4.urdf")
  set(KINOVA_URDF_PATH_BOTA_DS4 "${KINOVA_URDF_DIR}/gen3_7dof_novision_sim_bota_ds4.urdf")
  add_custom_command(
    OUTPUT ${GENERATED_URDF_BOTA_DS4}
    COMMAND ${XACRO} -o ${GENERATED_URDF_BOTA_DS4} -i ${KINOVA_XACRO_PATH} dof:=7 vision:=false bota:=true DS4:=true
    DEPENDS ${KINOVA_XACRO_FILES} ${BOTA_XACRO_FILES}
  )

  set(GENERATED_URDF_BOTA_FILES ${GENERATED_URDF} ${GENERATED_URDF_CAMERA} ${GENERATED_URDF_CAMERA})
endif()

# Custom target for generating URDF(s)
if(WITH_BOTA_SENSOR)
  add_custom_target(generate-kinova-urdf DEPENDS ${GENERATED_URDF_FILES} ${GENERATED_URDF_BOTA_FILES})
else()
  add_custom_target(generate-kinova-urdf DEPENDS ${GENERATED_URDF_FILES})
endif()

# Add source and install
add_subdirectory(src)

# Install generated URDF(s)
install(FILES ${GENERATED_URDF_FILES} DESTINATION ${KINOVA_URDF_DIR})
if(WITH_BOTA_SENSOR)
  install(FILES ${GENERATED_URDF_BOTA_FILES} DESTINATION ${KINOVA_URDF_DIR})
endif()

# Install convex directory
install(DIRECTORY convex DESTINATION ${CMAKE_INSTALL_DATADIR}/mc_kinova)
# Install rsdf directory
install(DIRECTORY rsdf DESTINATION ${CMAKE_INSTALL_DATADIR}/mc_kinova)

# Tests
if(BUILD_TESTING)
  add_subdirectory(tests)
endif()
