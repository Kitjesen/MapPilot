cmake_minimum_required(VERSION 3.16)

# Header-only INTERFACE library: unified DDS QoS profiles and topic contracts.
# Consumers link this target to get dds_qos_profiles.hpp and dds_topics.hpp
# without needing to manage CycloneDDS include paths manually.

if(NOT TARGET lingtu_dds_qos_profiles)
  add_library(lingtu_dds_qos_profiles INTERFACE)
  target_include_directories(lingtu_dds_qos_profiles INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/../..")
  target_link_libraries(lingtu_dds_qos_profiles INTERFACE CycloneDDS::ddsc)
endif()

# Generate and compile a CycloneDDS C type-support library once per build tree.
# Independent product build trees still generate their own copy; the IDL file is
# the source of truth shared by all of them.
function(lingtu_add_dds_c_messages target_name idl_file)
  if(TARGET "${target_name}")
    return()
  endif()
  if(NOT TARGET CycloneDDS::ddsc)
    message(FATAL_ERROR
      "${target_name} requires CycloneDDS::ddsc before message generation")
  endif()
  if(NOT EXISTS "${idl_file}")
    message(FATAL_ERROR "DDS IDL does not exist: ${idl_file}")
  endif()

  find_program(_LINGTU_IDLC_EXECUTABLE NAMES idlc REQUIRED)
  get_filename_component(_idl_stem "${idl_file}" NAME_WE)
  set(_generated_dir "${CMAKE_CURRENT_BINARY_DIR}/${target_name}_generated")
  set(_generated_header "${_generated_dir}/${_idl_stem}.h")
  set(_generated_source "${_generated_dir}/${_idl_stem}.c")
  file(MAKE_DIRECTORY "${_generated_dir}")

  add_custom_command(
    OUTPUT "${_generated_header}" "${_generated_source}"
    COMMAND "${_LINGTU_IDLC_EXECUTABLE}" -l c "${idl_file}"
    DEPENDS "${idl_file}"
    WORKING_DIRECTORY "${_generated_dir}"
    COMMENT "Generating CycloneDDS C types from ${idl_file}"
    VERBATIM)

  add_library("${target_name}" STATIC "${_generated_source}")
  set_target_properties("${target_name}" PROPERTIES
    POSITION_INDEPENDENT_CODE ON)
  target_include_directories("${target_name}" PUBLIC "${_generated_dir}")
  target_link_libraries("${target_name}" PUBLIC CycloneDDS::ddsc)
endfunction()
