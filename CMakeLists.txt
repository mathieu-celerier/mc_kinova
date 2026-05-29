cmake_minimum_required(VERSION 3.14)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 17)

set(PROJECT_NAME mc_kinova)
set(PROJECT_DESCRIPTION "KinovaRobotModule")
set(PROJECT_URL "")

include(GNUInstallDirs)

project(
  ${PROJECT_NAME}
  LANGUAGES CXX
  VERSION 1.0.0)
include(CTest)
enable_testing()

if(MSVC AND _VCPKG_INSTALLED_DIR)
  set(VCPKG_INSTALL_PREFIX "${_VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_PREFIX}")
  set(CMAKE_MSVCIDE_RUN_PATH
      "${VCPKG_INSTALL_PREFIX}/\$(Configuration)/bin;${VCPKG_INSTALL_PREFIX}/bin;${CMAKE_MSVCIDE_RUN_PATH}"
  )
endif()

find_package(mc_rtc REQUIRED)

# Default CMAKE_INSTALL_PREFIX to mc_rtc's prefix if user didn't set one
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  # mc_rtc_DIR = <prefix>/lib/cmake/mc_rtc → go up 3 levels
  get_filename_component(_mc_rtc_prefix "${mc_rtc_DIR}/../../.." ABSOLUTE)
  set(CMAKE_INSTALL_PREFIX
      "${_mc_rtc_prefix}"
      CACHE PATH "Install prefix (defaulting to mc_rtc's prefix)" FORCE)
  message(
    STATUS "CMAKE_INSTALL_PREFIX set to mc_rtc prefix: ${CMAKE_INSTALL_PREFIX}")
endif()

find_description_package(kortex_description)

# Find program xacro
find_program(XACRO xacro REQUIRED)

# Glob XACRO files for dependency tracking
file(GLOB_RECURSE KINOVA_XACRO_FILES "${KORTEX_DESCRIPTION_PATH}/*.xacro")

# Directory for generated URDF
set(KINOVA_URDF_DIR
    "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/urdf")
set(KINOVA_RSDF_DIR
    "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/rsdf")
set(KINOVA_XACRO_PATH "${CMAKE_CURRENT_BINARY_DIR}/xacro/gen3.xacro")
configure_file(${CMAKE_SOURCE_DIR}/xacro/gen3.in.xacro ${KINOVA_XACRO_PATH})

# Common URDF generation (no Bota sensor)
set(GENERATED_KINOVA_URDF "${CMAKE_CURRENT_BINARY_DIR}/urdf/kinova.urdf")
add_custom_command(
  OUTPUT ${GENERATED_KINOVA_URDF}
  COMMAND
    ${XACRO} -o ${GENERATED_KINOVA_URDF} -i
    ${KORTEX_DESCRIPTION_PATH}/robots/gen3.xacro dof:=7 vision:=false sim:=true
  DEPENDS ${KINOVA_XACRO_PATH} ${KINOVA_XACRO_FILES})

set(GENERATED_URDF_FILES ${GENERATED_KINOVA_URDF})

set(GENERATED_URDF_CAMERA "${CMAKE_CURRENT_BINARY_DIR}/urdf/kinova_camera.urdf")
add_custom_command(
  OUTPUT ${GENERATED_URDF_CAMERA}
  COMMAND ${XACRO} -o ${GENERATED_URDF_CAMERA} -i ${KINOVA_XACRO_PATH} dof:=7
          vision:=true bota:=false end_effector:=none
  DEPENDS ${KINOVA_XACRO_PATH} ${KINOVA_XACRO_FILES})
list(APPEND GENERATED_URDF_FILES ${GENERATED_URDF_CAMERA})

find_description_package(robotiq_description OPTIONAL)
if(${robotiq_description_FOUND})
  set(GENERATED_URDF_GRIPPER
      "${CMAKE_CURRENT_BINARY_DIR}/urdf/kinova_gripper.urdf")
  add_custom_command(
    OUTPUT ${GENERATED_URDF_GRIPPER}
    COMMAND
      ${XACRO} -o ${GENERATED_URDF_GRIPPER} -i ${KINOVA_XACRO_PATH} dof:=7
      vision:=false gripper:="robotiq_2f_85" bota:=false end_effector:=none
    DEPENDS ${KINOVA_XACRO_PATH} ${KINOVA_XACRO_FILES})
  list(APPEND GENERATED_URDF_FILES ${GENERATED_URDF_GRIPPER})

  set(GENERATED_URDF_CAMERA_GRIPPER
      "${CMAKE_CURRENT_BINARY_DIR}/urdf/kinova_camera_gripper.urdf")
  add_custom_command(
    OUTPUT ${GENERATED_URDF_CAMERA_GRIPPER}
    COMMAND
      ${XACRO} -o ${GENERATED_URDF_CAMERA_GRIPPER} -i ${KINOVA_XACRO_PATH}
      dof:=7 vision:=true gripper:="robotiq_2f_85" bota:=false
      end_effector:=none
    DEPENDS ${KINOVA_XACRO_PATH} ${KINOVA_XACRO_FILES})
  list(APPEND GENERATED_URDF_FILES ${GENERATED_URDF_CAMERA_GRIPPER})
else()
  message(STATUS "Robotiq gripper support DISABLED")
endif()

# Bota-specific URDF (only if WITH_BOTA_SENSOR)
find_description_package(bota_driver OPTIONAL)
if(${bota_driver_FOUND})
  set(WITH_BOTA_SENSOR ON)
  message(STATUS "Bota sensor support ENABLED")
  file(GLOB_RECURSE BOTA_XACRO_FILES "${bota_driver_DIR}/../urdf/*.xacro")
else()
  message(STATUS "Bota sensor support DISABLED")
endif()

if(WITH_BOTA_SENSOR)
  # Configure the composite xacro file (template-based)
  set(KINOVA_BOTA_XACRO_PATH
      "${CMAKE_CURRENT_BINARY_DIR}/xacro/gen3_bota.xacro")
  configure_file(${CMAKE_SOURCE_DIR}/xacro/gen3_bota.in.xacro
                 ${KINOVA_BOTA_XACRO_PATH})

  set(GENERATED_URDF_BOTA "${CMAKE_CURRENT_BINARY_DIR}/urdf/kinova_bota.urdf")
  add_custom_command(
    OUTPUT ${GENERATED_URDF_BOTA}
    COMMAND ${XACRO} -o ${GENERATED_URDF_BOTA} -i ${KINOVA_BOTA_XACRO_PATH}
            dof:=7 vision:=false end_effector:=none
    DEPENDS ${KINOVA_BOTA_XACRO_PATH} ${KINOVA_XACRO_FILES} ${BOTA_XACRO_FILES})
  list(APPEND GENERATED_URDF_FILES ${GENERATED_URDF_BOTA})

  set(GENERATED_URDF_BOTA_DS4
      "${CMAKE_CURRENT_BINARY_DIR}/urdf/kinova_bota_ds4.urdf")
  add_custom_command(
    OUTPUT ${GENERATED_URDF_BOTA_DS4}
    COMMAND ${XACRO} -o ${GENERATED_URDF_BOTA_DS4} -i ${KINOVA_BOTA_XACRO_PATH}
            dof:=7 vision:=false bota:=true end_effector:=ds4
    DEPENDS ${KINOVA_BOTA_XACRO_PATH} ${KINOVA_XACRO_FILES} ${BOTA_XACRO_FILES})
  list(APPEND GENERATED_URDF_FILES ${GENERATED_URDF_BOTA_DS4})

  set(GENERATED_URDF_BOTA_PLATE
      "${CMAKE_CURRENT_BINARY_DIR}/urdf/kinova_bota_plate.urdf")
  add_custom_command(
    OUTPUT ${GENERATED_URDF_BOTA_PLATE}
    COMMAND
      ${XACRO} -o ${GENERATED_URDF_BOTA_PLATE} -i ${KINOVA_BOTA_XACRO_PATH}
      dof:=7 vision:=false bota:=true end_effector:=plate
    DEPENDS ${KINOVA_BOTA_XACRO_PATH} ${KINOVA_XACRO_FILES} ${BOTA_XACRO_FILES})
  list(APPEND GENERATED_URDF_FILES ${GENERATED_URDF_BOTA_PLATE})

  set(GENERATED_URDF_BOTA_SCREW
      "${CMAKE_CURRENT_BINARY_DIR}/urdf/kinova_bota_screw.urdf")
  add_custom_command(
    OUTPUT ${GENERATED_URDF_BOTA_SCREW}
    COMMAND
      ${XACRO} -o ${GENERATED_URDF_BOTA_SCREW} -i ${KINOVA_BOTA_XACRO_PATH}
      dof:=7 vision:=false bota:=true end_effector:=screw
    DEPENDS ${KINOVA_BOTA_XACRO_PATH} ${KINOVA_XACRO_FILES} ${BOTA_XACRO_FILES})
  list(APPEND GENERATED_URDF_FILES ${GENERATED_URDF_BOTA_SCREW})
endif()

# Custom target for generating URDF(s)
add_custom_target(generate-kinova-urdf DEPENDS ${GENERATED_URDF_FILES})

# Add source and install
add_subdirectory(src)

# Install generated URDFs and RSDFs
install(FILES ${GENERATED_URDF_FILES} DESTINATION ${KINOVA_URDF_DIR})
install(DIRECTORY rsdf/ DESTINATION ${KINOVA_RSDF_DIR})

install(
  DIRECTORY etc/ForceSensorCalibration/
  DESTINATION ${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/ForceSensorCalibration/
  FILES_MATCHING
  PATTERN "*.yaml")

# Tests
if(BUILD_TESTING)
  add_subdirectory(tests)
endif()
