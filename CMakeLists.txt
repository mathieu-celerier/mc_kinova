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
file(GLOB_RECURSE LOCAL_KINOVA_XACRO_FILES CONFIGURE_DEPENDS
     "${CMAKE_SOURCE_DIR}/xacro/*")
file(GLOB_RECURSE KINOVA_XACRO_FILES CONFIGURE_DEPENDS
     "${KORTEX_DESCRIPTION_PATH}/*.xacro")
set(LOCAL_BOTA_XACRO_TEMPLATE
    "${CMAKE_SOURCE_DIR}/xacro/BFT_KG3_IND2_SW.urdf.xacro")

# Bota sensor integration
find_description_package(rokubimini_description OPTIONAL)
if(${rokubimini_description_FOUND})
  message(STATUS "Bota sensor support ENABLED")
  set(WITH_BOTA_SENSOR_GEN0 ON)
  file(GLOB_RECURSE BOTA_XACRO_FILES CONFIGURE_DEPENDS
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

set(KORTEX_GEN3_MACRO_PATH
    "${KORTEX_DESCRIPTION_PATH}/arms/gen3/7dof/urdf/gen3_macro.xacro")
file(READ "${KORTEX_GEN3_MACRO_PATH}" KORTEX_GEN3_MACRO_CONTENTS)
string(REGEX MATCH "<xacro:macro[^\n]*name=\"load_arm\"[^\"]*params=\"([^\"]|\n|\r)*\""
       KORTEX_LOAD_ARM_MACRO_DECL "${KORTEX_GEN3_MACRO_CONTENTS}")

set(KORTEX_GRIPPER_MACRO_PATH
    "${KORTEX_DESCRIPTION_PATH}/grippers/robotiq_2f_85/urdf/robotiq_2f_85_macro.xacro")
file(READ "${KORTEX_GRIPPER_MACRO_PATH}" KORTEX_GRIPPER_MACRO_CONTENTS)
string(REGEX MATCH "<xacro:macro[^\n]*name=\"load_gripper\"[^\"]*params=\"([^\"]|\n|\r)*\""
       KORTEX_LOAD_GRIPPER_MACRO_DECL "${KORTEX_GRIPPER_MACRO_CONTENTS}")

if(KORTEX_LOAD_ARM_MACRO_DECL MATCHES "[\r\n][ \t]*gripper([:= \t\r\n]|$)")
  set(KINOVA_LOAD_ARM_GRIPPER_ARGS
"    gripper=\"$(arg gripper)\"
    gripper_joint_name=\"$(arg gripper_joint_name)\"
    gripper_max_velocity=\"$(arg gripper_max_velocity)\"
    gripper_max_force=\"$(arg gripper_max_force)\"")
elseif(KORTEX_LOAD_ARM_MACRO_DECL MATCHES "[\r\n][ \t]*gripper_joint_name([:= \t\r\n]|$)")
  set(KINOVA_LOAD_ARM_GRIPPER_ARGS
"    gripper_joint_name=\"$(arg gripper_joint_name)\"
    gripper_max_velocity=\"$(arg gripper_max_velocity)\"
    gripper_max_force=\"$(arg gripper_max_force)\"")
else()
  set(KINOVA_LOAD_ARM_GRIPPER_ARGS "")
endif()

if(KORTEX_LOAD_ARM_MACRO_DECL MATCHES "[\r\n][ \t]*use_internal_bus_gripper_comm([:= \t\r\n]|$)")
  set(KINOVA_LOAD_ARM_INTERNAL_BUS_ARG
"    use_internal_bus_gripper_comm=\"$(arg use_internal_bus_gripper_comm)\"")
else()
  set(KINOVA_LOAD_ARM_INTERNAL_BUS_ARG "")
endif()

set(KINOVA_LOAD_GRIPPER_ARGS
"      parent=\"@PARENT@\"
      prefix=\"$(arg prefix)\"
      use_fake_hardware=\"$(arg use_fake_hardware)\"
      fake_sensor_commands=\"$(arg fake_sensor_commands)\"")
if(KORTEX_LOAD_GRIPPER_MACRO_DECL MATCHES "[\r\n][ \t]*sim_gazebo([:= \t\r\n]|$)")
  string(APPEND KINOVA_LOAD_GRIPPER_ARGS
"\n      sim_gazebo=\"$(arg sim_gazebo)\"")
endif()
if(KORTEX_LOAD_GRIPPER_MACRO_DECL MATCHES "[\r\n][ \t]*sim_isaac([:= \t\r\n]|$)")
  string(APPEND KINOVA_LOAD_GRIPPER_ARGS
"\n      sim_isaac=\"$(arg sim_isaac)\"")
endif()
if(KORTEX_LOAD_GRIPPER_MACRO_DECL MATCHES "[\r\n][ \t]*com_port([:= \t\r\n]|$)")
  string(APPEND KINOVA_LOAD_GRIPPER_ARGS
"\n      com_port=\"/dev/ttyUSB0\"")
endif()
if(KORTEX_LOAD_GRIPPER_MACRO_DECL MATCHES "[\r\n][ \t]*use_internal_bus_gripper_comm([:= \t\r\n]|$)")
  string(APPEND KINOVA_LOAD_GRIPPER_ARGS
"\n      use_internal_bus_gripper_comm=\"$(arg use_internal_bus_gripper_comm)\"")
endif()
string(REPLACE "@PARENT@" "$(arg prefix)end_effector_link"
       KINOVA_LOAD_GRIPPER_ARGS "${KINOVA_LOAD_GRIPPER_ARGS}")
set(KINOVA_BOTA_LOAD_GRIPPER_ARGS "${KINOVA_LOAD_GRIPPER_ARGS}")
string(REPLACE "parent=\"$(arg prefix)end_effector_link\"" "parent=\"tool\""
       KINOVA_BOTA_LOAD_GRIPPER_ARGS "${KINOVA_BOTA_LOAD_GRIPPER_ARGS}")

set(KINOVA_XACRO_PATH "${CMAKE_CURRENT_BINARY_DIR}/xacro/gen3.xacro")
configure_file(${CMAKE_SOURCE_DIR}/xacro/gen3.in.xacro ${KINOVA_XACRO_PATH})

# force-mc_kinova in the superbuild removes the configure stamp and reruns CMake.
# Refresh this file on each configure so all URDF custom commands can be forced.
string(TIMESTAMP KINOVA_URDF_GENERATION_TOKEN "%Y-%m-%dT%H:%M:%SZ" UTC)
set(KINOVA_URDF_RECONFIGURE_STAMP
    "${CMAKE_CURRENT_BINARY_DIR}/urdf/.reconfigure.stamp")
file(WRITE "${KINOVA_URDF_RECONFIGURE_STAMP}"
     "${KINOVA_URDF_GENERATION_TOKEN}\n")
set(KINOVA_URDF_DEPENDS
    ${KINOVA_URDF_RECONFIGURE_STAMP}
    ${LOCAL_KINOVA_XACRO_FILES}
    ${KINOVA_XACRO_FILES}
    ${BOTA_XACRO_FILES})

function(add_kinova_urdf output_name)
  set(options)
  set(oneValueArgs VISION GRIPPER BOTA END_EFFECTOR)
  cmake_parse_arguments(KINOVA_URDF "${options}" "${oneValueArgs}" "" ${ARGN})

  set(output_path "${CMAKE_CURRENT_BINARY_DIR}/urdf/${output_name}.urdf")
  add_custom_command(
    OUTPUT ${output_path}
    COMMAND ${XACRO} -o ${output_path} -i ${KINOVA_XACRO_PATH}
            dof:=7
            vision:=${KINOVA_URDF_VISION}
            gripper:="${KINOVA_URDF_GRIPPER}"
            bota:=${KINOVA_URDF_BOTA}
            end_effector:=${KINOVA_URDF_END_EFFECTOR}
    DEPENDS ${KINOVA_XACRO_PATH} ${KINOVA_URDF_DEPENDS})
  set(${output_name}_URDF ${output_path} PARENT_SCOPE)
endfunction()

add_kinova_urdf(kinova VISION false GRIPPER "" BOTA false END_EFFECTOR none)
add_kinova_urdf(kinova_camera VISION true GRIPPER "" BOTA false END_EFFECTOR none)
add_kinova_urdf(kinova_camera_gripper VISION true GRIPPER "robotiq_2f_85" BOTA false END_EFFECTOR none)
add_kinova_urdf(kinova_gripper VISION false GRIPPER "robotiq_2f_85" BOTA false END_EFFECTOR none)
add_kinova_urdf(kinova_camera_gripper_2f140 VISION true GRIPPER "robotiq_2f_140" BOTA false END_EFFECTOR none)
add_kinova_urdf(kinova_gripper_2f140 VISION false GRIPPER "robotiq_2f_140" BOTA false END_EFFECTOR none)

set(GENERATED_URDF_FILES
    ${kinova_URDF}
    ${kinova_camera_URDF}
    ${kinova_camera_gripper_URDF}
    ${kinova_gripper_URDF}
    ${kinova_camera_gripper_2f140_URDF}
    ${kinova_gripper_2f140_URDF})


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
    DEPENDS ${KINOVA_XACRO_PATH} ${KINOVA_URDF_DEPENDS}
  )

  set(GENERATED_URDF_BOTA_DS4 "${CMAKE_CURRENT_BINARY_DIR}/urdf/kinova_bota_ds4.urdf")
  add_custom_command(
    OUTPUT ${GENERATED_URDF_BOTA_DS4}
    COMMAND ${XACRO} -o ${GENERATED_URDF_BOTA_DS4} -i ${KINOVA_XACRO_PATH} dof:=7 vision:=false bota:=true end_effector:=ds4
    DEPENDS ${KINOVA_XACRO_PATH} ${KINOVA_URDF_DEPENDS}
  )
  
  set(GENERATED_URDF_BOTA_PLATE "${CMAKE_CURRENT_BINARY_DIR}/urdf/kinova_bota_plate.urdf")
  add_custom_command(
    OUTPUT ${GENERATED_URDF_BOTA_PLATE}
    COMMAND ${XACRO} -o ${GENERATED_URDF_BOTA_PLATE} -i ${KINOVA_XACRO_PATH} dof:=7 vision:=false bota:=true end_effector:=plate
    DEPENDS ${KINOVA_XACRO_PATH} ${KINOVA_URDF_DEPENDS}
  )

  set(GENERATED_URDF_BOTA_SCREW "${CMAKE_CURRENT_BINARY_DIR}/urdf/kinova_bota_screw.urdf")
  add_custom_command(
    OUTPUT ${GENERATED_URDF_BOTA_SCREW}
    COMMAND ${XACRO} -o ${GENERATED_URDF_BOTA_SCREW} -i ${KINOVA_XACRO_PATH} dof:=7 vision:=false bota:=true end_effector:=screw
    DEPENDS ${KINOVA_XACRO_PATH} ${KINOVA_URDF_DEPENDS}
  )

  set(GENERATED_URDF_BOTA_FILES ${GENERATED_URDF_BOTA} ${GENERATED_URDF_BOTA_DS4} ${GENERATED_URDF_BOTA_PLATE} ${GENERATED_URDF_BOTA_SCREW})
endif()

if(WITH_BOTA_SENSOR_GENA)
  set(GENERATED_URDF_BOTA_GENA "${CMAKE_CURRENT_BINARY_DIR}/urdf/kinova_bota_gena.urdf")
  add_custom_command(
    OUTPUT ${GENERATED_URDF_BOTA_GENA}
    COMMAND ${XACRO} -o ${GENERATED_URDF_BOTA_GENA} -i ${KINOVA_XACRO_PATH} dof:=7 vision:=false bota:=true bota_version:=gena end_effector:=none
    DEPENDS ${KINOVA_XACRO_PATH} ${KINOVA_URDF_DEPENDS} ${LOCAL_BOTA_XACRO_TEMPLATE}
  )

  set(GENERATED_URDF_BOTA_GENA_DS4 "${CMAKE_CURRENT_BINARY_DIR}/urdf/kinova_bota_gena_ds4.urdf")
  add_custom_command(
    OUTPUT ${GENERATED_URDF_BOTA_GENA_DS4}
    COMMAND ${XACRO} -o ${GENERATED_URDF_BOTA_GENA_DS4} -i ${KINOVA_XACRO_PATH} dof:=7 vision:=false bota:=true bota_version:=gena end_effector:=ds4
    DEPENDS ${KINOVA_XACRO_PATH} ${KINOVA_URDF_DEPENDS} ${LOCAL_BOTA_XACRO_TEMPLATE}
  )

  set(GENERATED_URDF_BOTA_GENA_PLATE "${CMAKE_CURRENT_BINARY_DIR}/urdf/kinova_bota_gena_plate.urdf")
  add_custom_command(
    OUTPUT ${GENERATED_URDF_BOTA_GENA_PLATE}
    COMMAND ${XACRO} -o ${GENERATED_URDF_BOTA_GENA_PLATE} -i ${KINOVA_XACRO_PATH} dof:=7 vision:=false bota:=true bota_version:=gena end_effector:=plate
    DEPENDS ${KINOVA_XACRO_PATH} ${KINOVA_URDF_DEPENDS} ${LOCAL_BOTA_XACRO_TEMPLATE}
  )

  set(GENERATED_URDF_BOTA_GENA_SCREW "${CMAKE_CURRENT_BINARY_DIR}/urdf/kinova_bota_gena_screw.urdf")
  add_custom_command(
    OUTPUT ${GENERATED_URDF_BOTA_GENA_SCREW}
    COMMAND ${XACRO} -o ${GENERATED_URDF_BOTA_GENA_SCREW} -i ${KINOVA_XACRO_PATH} dof:=7 vision:=false bota:=true bota_version:=gena end_effector:=screw
    DEPENDS ${KINOVA_XACRO_PATH} ${KINOVA_URDF_DEPENDS} ${LOCAL_BOTA_XACRO_TEMPLATE}
  )

  set(GENERATED_URDF_BOTA_GENA_GRIPPER "${CMAKE_CURRENT_BINARY_DIR}/urdf/kinova_bota_gena_gripper.urdf")
  add_custom_command(
    OUTPUT ${GENERATED_URDF_BOTA_GENA_GRIPPER}
    COMMAND ${XACRO} -o ${GENERATED_URDF_BOTA_GENA_GRIPPER} -i ${KINOVA_XACRO_PATH} dof:=7 vision:=false bota:=true bota_version:=gena end_effector:=none gripper:="robotiq_2f_85"
    DEPENDS ${KINOVA_XACRO_PATH} ${KINOVA_URDF_DEPENDS} ${LOCAL_BOTA_XACRO_TEMPLATE}
  )

  set(GENERATED_URDF_BOTA_GENA_GRIPPER_2F140 "${CMAKE_CURRENT_BINARY_DIR}/urdf/kinova_bota_gena_gripper_2f140.urdf")
  add_custom_command(
    OUTPUT ${GENERATED_URDF_BOTA_GENA_GRIPPER_2F140}
    COMMAND ${XACRO} -o ${GENERATED_URDF_BOTA_GENA_GRIPPER_2F140} -i ${KINOVA_XACRO_PATH} dof:=7 vision:=false bota:=true bota_version:=gena end_effector:=none gripper:="robotiq_2f_140"
    DEPENDS ${KINOVA_XACRO_PATH} ${KINOVA_URDF_DEPENDS} ${LOCAL_BOTA_XACRO_TEMPLATE}
  )

  set(GENERATED_URDF_BOTA_GENA_FILES
      ${GENERATED_URDF_BOTA_GENA}
      ${GENERATED_URDF_BOTA_GENA_DS4}
      ${GENERATED_URDF_BOTA_GENA_PLATE}
      ${GENERATED_URDF_BOTA_GENA_SCREW}
      ${GENERATED_URDF_BOTA_GENA_GRIPPER}
      ${GENERATED_URDF_BOTA_GENA_GRIPPER_2F140})
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
