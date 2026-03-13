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

find_description_package(kortex_description)

# Find program xacro
find_program(XACRO xacro REQUIRED)

# Glob XACRO files for dependency tracking
file(GLOB_RECURSE KINOVA_XACRO_FILES "${KORTEX_DESCRIPTION_PATH}/*.xacro")
set(LOCAL_BOTA_XACRO_TEMPLATE
    "${CMAKE_SOURCE_DIR}/xacro/BFT_KG3_IND2_SW.urdf.xacro")

# Bota sensor integration
find_description_package(rokubimini_description OPTIONAL)
if(${rokubimini_description_FOUND})
  message(STATUS "Bota sensor support ENABLED")
  set(WITH_BOTA_SENSOR_GEN0 ON)
  file(GLOB_RECURSE BOTA_XACRO_FILES
       "${rokubimini_description_DIR}/../urdf/*.xacro")
else()
  message(STATUS "Bota Gen0 sensor support DISABLED")
endif()
message(STATUS "Bota GenA sensor support ENABLED")
set(WITH_BOTA_SENSOR_GENA ON)

# Directory for generated URDF
set(KINOVA_URDF_DIR "${CMAKE_INSTALL_PREFIX}/share/mc_kinova/urdf")
set(KINOVA_RSDF_DIR "${CMAKE_INSTALL_PREFIX}/share/mc_kinova/rsdf")
set(KINOVA_CONVEX_DIR "${CMAKE_INSTALL_PREFIX}/share/mc_kinova/convex")
set(KINOVA_BOTA_GENA_XACRO_PATH
    "${CMAKE_CURRENT_BINARY_DIR}/xacro/BFT_KG3_IND2_SW.urdf.xacro")
configure_file(${LOCAL_BOTA_XACRO_TEMPLATE} ${KINOVA_BOTA_GENA_XACRO_PATH} @ONLY)
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

set(GENERATED_URDF_CAMERA "${CMAKE_CURRENT_BINARY_DIR}/urdf/kinova_camera.urdf")
add_custom_command(
  OUTPUT ${GENERATED_URDF_CAMERA}
  COMMAND ${XACRO} -o ${GENERATED_URDF_CAMERA} -i ${KINOVA_XACRO_PATH} dof:=7 vision:=true bota:=false end_effector:=none
  DEPENDS ${KINOVA_XACRO_PATH} ${KINOVA_XACRO_FILES} ${BOTA_XACRO_FILES}
)

set(GENERATED_URDF_CAMERA_GRIPPER "${CMAKE_CURRENT_BINARY_DIR}/urdf/kinova_camera_gripper.urdf")
add_custom_command(
  OUTPUT ${GENERATED_URDF_CAMERA_GRIPPER}
  COMMAND ${XACRO} -o ${GENERATED_URDF_CAMERA_GRIPPER} -i ${KINOVA_XACRO_PATH} dof:=7 vision:=true gripper:="robotiq_2f_85" bota:=false end_effector:=none
  DEPENDS ${KINOVA_XACRO_PATH} ${KINOVA_XACRO_FILES} ${BOTA_XACRO_FILES}
)

set(GENERATED_URDF_GRIPPER "${CMAKE_CURRENT_BINARY_DIR}/urdf/kinova_gripper.urdf")
add_custom_command(
  OUTPUT ${GENERATED_URDF_GRIPPER}
  COMMAND ${XACRO} -o ${GENERATED_URDF_GRIPPER} -i ${KINOVA_XACRO_PATH} dof:=7 vision:=false gripper:="robotiq_2f_85" bota:=false end_effector:=none
  DEPENDS ${KINOVA_XACRO_PATH} ${KINOVA_XACRO_FILES} ${BOTA_XACRO_FILES}
)

set(GENERATED_URDF_FILES ${GENERATED_KINOVA_URDF} ${GENERATED_URDF_CAMERA} ${GENERATED_URDF_CAMERA_GRIPPER} ${GENERATED_URDF_GRIPPER})


# Bota-specific URDF (only if WITH_BOTA_SENSOR)
if(WITH_BOTA_SENSOR_GEN0 OR WITH_BOTA_SENSOR_GENA)
  # Configure the composite xacro file (template-based)
  set(KINOVA_XACRO_PATH "${CMAKE_CURRENT_BINARY_DIR}/xacro/gen3_bota.xacro")
  configure_file(${CMAKE_SOURCE_DIR}/xacro/gen3_bota.in.xacro ${KINOVA_XACRO_PATH})
endif()

if(WITH_BOTA_SENSOR_GEN0)
  set(GENERATED_URDF_BOTA "${CMAKE_BINARY_DIR}/urdf/kinova_bota.urdf")
  add_custom_command(
    OUTPUT ${GENERATED_URDF_BOTA}
    COMMAND ${XACRO} -o ${GENERATED_URDF_BOTA} -i ${KINOVA_XACRO_PATH} dof:=7 vision:=false end_effector:=none
    DEPENDS ${KINOVA_XACRO_PATH} ${KINOVA_XACRO_FILES} ${BOTA_XACRO_FILES}
  )

  set(GENERATED_URDF_BOTA_DS4 "${CMAKE_CURRENT_BINARY_DIR}/urdf/kinova_bota_ds4.urdf")
  add_custom_command(
    OUTPUT ${GENERATED_URDF_BOTA_DS4}
    COMMAND ${XACRO} -o ${GENERATED_URDF_BOTA_DS4} -i ${KINOVA_XACRO_PATH} dof:=7 vision:=false bota:=true end_effector:=ds4
    DEPENDS ${KINOVA_XACRO_PATH} ${KINOVA_XACRO_FILES} ${BOTA_XACRO_FILES}
  )
  
  set(GENERATED_URDF_BOTA_PLATE "${CMAKE_CURRENT_BINARY_DIR}/urdf/kinova_bota_plate.urdf")
  add_custom_command(
    OUTPUT ${GENERATED_URDF_BOTA_PLATE}
    COMMAND ${XACRO} -o ${GENERATED_URDF_BOTA_PLATE} -i ${KINOVA_XACRO_PATH} dof:=7 vision:=false bota:=true end_effector:=plate
    DEPENDS ${KINOVA_XACRO_PATH} ${KINOVA_XACRO_FILES} ${BOTA_XACRO_FILES}
  )

  set(GENERATED_URDF_BOTA_SCREW "${CMAKE_CURRENT_BINARY_DIR}/urdf/kinova_bota_screw.urdf")
  add_custom_command(
    OUTPUT ${GENERATED_URDF_BOTA_SCREW}
    COMMAND ${XACRO} -o ${GENERATED_URDF_BOTA_SCREW} -i ${KINOVA_XACRO_PATH} dof:=7 vision:=false bota:=true end_effector:=screw
    DEPENDS ${KINOVA_XACRO_PATH} ${KINOVA_XACRO_FILES} ${BOTA_XACRO_FILES}
  )

  set(GENERATED_URDF_BOTA_FILES ${GENERATED_URDF_BOTA} ${GENERATED_URDF_BOTA_DS4} ${GENERATED_URDF_BOTA_PLATE} ${GENERATED_URDF_BOTA_SCREW})
endif()

if(WITH_BOTA_SENSOR_GENA)
  set(GENERATED_URDF_BOTA_GENA "${CMAKE_CURRENT_BINARY_DIR}/urdf/kinova_bota_gena.urdf")
  add_custom_command(
    OUTPUT ${GENERATED_URDF_BOTA_GENA}
    COMMAND ${XACRO} -o ${GENERATED_URDF_BOTA_GENA} -i ${KINOVA_XACRO_PATH} dof:=7 vision:=false bota:=true bota_version:=gena end_effector:=none
    DEPENDS ${KINOVA_XACRO_PATH} ${KINOVA_XACRO_FILES} ${LOCAL_BOTA_XACRO_TEMPLATE}
  )

  set(GENERATED_URDF_BOTA_GENA_DS4 "${CMAKE_CURRENT_BINARY_DIR}/urdf/kinova_bota_gena_ds4.urdf")
  add_custom_command(
    OUTPUT ${GENERATED_URDF_BOTA_GENA_DS4}
    COMMAND ${XACRO} -o ${GENERATED_URDF_BOTA_GENA_DS4} -i ${KINOVA_XACRO_PATH} dof:=7 vision:=false bota:=true bota_version:=gena end_effector:=ds4
    DEPENDS ${KINOVA_XACRO_PATH} ${KINOVA_XACRO_FILES} ${LOCAL_BOTA_XACRO_TEMPLATE}
  )

  set(GENERATED_URDF_BOTA_GENA_PLATE "${CMAKE_CURRENT_BINARY_DIR}/urdf/kinova_bota_gena_plate.urdf")
  add_custom_command(
    OUTPUT ${GENERATED_URDF_BOTA_GENA_PLATE}
    COMMAND ${XACRO} -o ${GENERATED_URDF_BOTA_GENA_PLATE} -i ${KINOVA_XACRO_PATH} dof:=7 vision:=false bota:=true bota_version:=gena end_effector:=plate
    DEPENDS ${KINOVA_XACRO_PATH} ${KINOVA_XACRO_FILES} ${LOCAL_BOTA_XACRO_TEMPLATE}
  )

  set(GENERATED_URDF_BOTA_GENA_SCREW "${CMAKE_CURRENT_BINARY_DIR}/urdf/kinova_bota_gena_screw.urdf")
  add_custom_command(
    OUTPUT ${GENERATED_URDF_BOTA_GENA_SCREW}
    COMMAND ${XACRO} -o ${GENERATED_URDF_BOTA_GENA_SCREW} -i ${KINOVA_XACRO_PATH} dof:=7 vision:=false bota:=true bota_version:=gena end_effector:=screw
    DEPENDS ${KINOVA_XACRO_PATH} ${KINOVA_XACRO_FILES} ${LOCAL_BOTA_XACRO_TEMPLATE}
  )

  set(GENERATED_URDF_BOTA_GENA_FILES
      ${GENERATED_URDF_BOTA_GENA}
      ${GENERATED_URDF_BOTA_GENA_DS4}
      ${GENERATED_URDF_BOTA_GENA_PLATE}
      ${GENERATED_URDF_BOTA_GENA_SCREW})
endif()

# Custom target for generating URDF(s)
if(WITH_BOTA_SENSOR_GEN0 OR WITH_BOTA_SENSOR_GENA)
  add_custom_target(generate-kinova-urdf DEPENDS ${GENERATED_URDF_FILES} ${GENERATED_URDF_BOTA_FILES} ${GENERATED_URDF_BOTA_GENA_FILES})
else()
  add_custom_target(generate-kinova-urdf DEPENDS ${GENERATED_URDF_FILES})
endif()

# Add source and install
add_subdirectory(src)

# Install generated URDF(s)
install(FILES ${GENERATED_URDF_FILES} DESTINATION ${KINOVA_URDF_DIR})
if(WITH_BOTA_SENSOR_GEN0)
  install(FILES ${GENERATED_URDF_BOTA_FILES} DESTINATION ${KINOVA_URDF_DIR})
endif()
if(WITH_BOTA_SENSOR_GENA)
  install(FILES ${GENERATED_URDF_BOTA_GENA_FILES} DESTINATION ${KINOVA_URDF_DIR})
endif()

# Install convex directory
install(DIRECTORY convex DESTINATION ${CMAKE_INSTALL_DATADIR}/mc_kinova)
# Install rsdf directory
install(DIRECTORY rsdf DESTINATION ${CMAKE_INSTALL_DATADIR}/mc_kinova)

# Tests
if(BUILD_TESTING)
  add_subdirectory(tests)
endif()

install(DIRECTORY etc/ForceSensorCalibration/ DESTINATION ${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/ForceSensorCalibration/
        FILES_MATCHING PATTERN "*.yaml")
