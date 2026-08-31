cmake_minimum_required(VERSION 3.16)

# Header-only INTERFACE library: unified DDS QoS profiles and topic contracts.
# Consumers link this target to get qos.hpp and topics.hpp
# without needing to manage CycloneDDS include paths manually.

if(NOT TARGET lingtu_dds_contracts)
  add_library(lingtu_dds_contracts INTERFACE)
  target_include_directories(lingtu_dds_contracts INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/../..")
  target_link_libraries(lingtu_dds_contracts INTERFACE CycloneDDS::ddsc)
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

  set(_idlc_command
    "${_LINGTU_IDLC_EXECUTABLE}" -l c "${idl_file}")
  if(UNIX)
    # idlc is a build tool, not part of the robot process.  Give it only the
    # CycloneDDS libraries installed beside that executable so an inherited
    # SDK2/runtime LD_LIBRARY_PATH cannot load an ABI-incompatible libddsc.
    get_filename_component(_idlc_bin_dir "${_LINGTU_IDLC_EXECUTABLE}" DIRECTORY)
    get_filename_component(_idlc_prefix "${_idlc_bin_dir}" DIRECTORY)
    set(_idlc_runtime_dirs)
    foreach(_idlc_lib_dir IN ITEMS
        "${_idlc_prefix}/lib"
        "${_idlc_prefix}/lib64")
      if(IS_DIRECTORY "${_idlc_lib_dir}")
        list(APPEND _idlc_runtime_dirs "${_idlc_lib_dir}")
      endif()
    endforeach()
    if(CMAKE_LIBRARY_ARCHITECTURE)
      set(_idlc_multiarch_lib_dir
        "${_idlc_prefix}/lib/${CMAKE_LIBRARY_ARCHITECTURE}")
      if(IS_DIRECTORY "${_idlc_multiarch_lib_dir}")
        list(APPEND _idlc_runtime_dirs "${_idlc_multiarch_lib_dir}")
      endif()
    endif()
    list(REMOVE_DUPLICATES _idlc_runtime_dirs)
    list(JOIN _idlc_runtime_dirs ":" _idlc_library_path)
    set(_idlc_command
      "${CMAKE_COMMAND}" -E env
      "LD_LIBRARY_PATH=${_idlc_library_path}"
      "${_LINGTU_IDLC_EXECUTABLE}" -l c "${idl_file}")
  endif()

  add_custom_command(
    OUTPUT "${_generated_header}" "${_generated_source}"
    COMMAND ${_idlc_command}
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
