cmake_minimum_required(VERSION 3.16)
project(lingtu_driver LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(CTest)

get_filename_component(_LINGTU_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../../.." ABSOLUTE)
set(_LINGTU_DRIVER_TEST_DIR "${_LINGTU_ROOT}/tests/drivers/real/motion")

if(MSVC)
  set(_LINGTU_DRIVER_WARNINGS /W4)
else()
  set(_LINGTU_DRIVER_WARNINGS -Wall -Wextra -Wpedantic)
endif()

add_library(lingtu_driver_core STATIC
  cmd_vel_writer_gate.cpp
  command_freshness_gate.cpp
  core.cpp
  config.cpp)
target_include_directories(lingtu_driver_core PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_options(lingtu_driver_core PRIVATE ${_LINGTU_DRIVER_WARNINGS})

if(BUILD_TESTING)
  add_executable(test_output_ack
    "${_LINGTU_DRIVER_TEST_DIR}/test_output_ack.cpp")
  target_include_directories(test_output_ack PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
  target_compile_options(test_output_ack PRIVATE ${_LINGTU_DRIVER_WARNINGS})
  add_test(NAME test_output_ack COMMAND test_output_ack)

  add_executable(test_driver_core
    "${_LINGTU_DRIVER_TEST_DIR}/test_core.cpp")
  target_link_libraries(test_driver_core PRIVATE lingtu_driver_core)
  target_compile_options(test_driver_core PRIVATE ${_LINGTU_DRIVER_WARNINGS})
  add_test(NAME test_driver_core COMMAND test_driver_core)

  add_executable(test_driver_status
    "${_LINGTU_DRIVER_TEST_DIR}/test_status.cpp"
    status.cpp)
  target_include_directories(test_driver_status PRIVATE "${_LINGTU_ROOT}/src")
  target_link_libraries(test_driver_status PRIVATE lingtu_driver_core)
  target_compile_options(test_driver_status PRIVATE ${_LINGTU_DRIVER_WARNINGS})
  add_test(NAME test_driver_status COMMAND test_driver_status)
endif()

if(UNIX)
  set(_DRIVER_RUNTIME_DEFAULT ON)
else()
  set(_DRIVER_RUNTIME_DEFAULT OFF)
endif()
option(LINGTU_DRIVER_BUILD_RUNTIME "Build the native field motion driver" ${_DRIVER_RUNTIME_DEFAULT})
set(LINGTU_DRIVER_BACKEND "" CACHE STRING "Real robot driver: doso or go2")
set_property(CACHE LINGTU_DRIVER_BACKEND PROPERTY STRINGS doso go2)

set(_LINGTU_DRIVER_ROBOTS doso go2)

if(LINGTU_DRIVER_BUILD_RUNTIME)
  if(NOT LINGTU_DRIVER_BACKEND IN_LIST _LINGTU_DRIVER_ROBOTS)
    message(FATAL_ERROR
      "LINGTU_DRIVER_BACKEND must be doso or go2; got '${LINGTU_DRIVER_BACKEND}'")
  endif()
  if(NOT UNIX)
    message(FATAL_ERROR "lingtu_driver runtime is supported on Linux; build only tests on this host")
  endif()
  if(LINGTU_DRIVER_BACKEND STREQUAL "go2")
    set(LINGTU_UNITREE_SDK2_PREFIX "$ENV{LINGTU_UNITREE_SDK2_PREFIX}"
      CACHE PATH "Installed Unitree SDK2 prefix")
    if(NOT LINGTU_UNITREE_SDK2_PREFIX)
      set(LINGTU_UNITREE_SDK2_PREFIX "/opt/unitree_robotics")
    endif()
    list(PREPEND CMAKE_PREFIX_PATH "${LINGTU_UNITREE_SDK2_PREFIX}")
    find_package(unitree_sdk2 REQUIRED CONFIG)
    if(NOT TARGET CycloneDDS::ddsc)
      add_library(CycloneDDS::ddsc ALIAS ddsc)
    endif()
  else()
    find_package(CycloneDDS REQUIRED)
  endif()
  add_subdirectory(
    "${_LINGTU_ROOT}/src/message/cpp"
    "${CMAKE_CURRENT_BINARY_DIR}/message_cpp")
  set(_LINGTU_IDL "${_LINGTU_ROOT}/src/message/idl/messages.idl")
  lingtu_add_dds_c_messages(lingtu_driver_messages "${_LINGTU_IDL}")

  add_library(lingtu_driver_io STATIC
    dds.cpp)
  target_include_directories(lingtu_driver_io PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${_LINGTU_ROOT}/src")
  target_link_libraries(lingtu_driver_io PUBLIC
    lingtu_driver_messages
    lingtu_dds_contracts
    CycloneDDS::ddsc)

  if(LINGTU_DRIVER_BACKEND STREQUAL "doso")
    set(LINGTU_BRAINSTEM_CLIENT_PREFIX "$ENV{LINGTU_BRAINSTEM_CLIENT_PREFIX}"
      CACHE PATH "Installed Brainstem C++ Client SDK prefix")
    if(LINGTU_BRAINSTEM_CLIENT_PREFIX)
      list(PREPEND CMAKE_PREFIX_PATH "${LINGTU_BRAINSTEM_CLIENT_PREFIX}")
    endif()
    find_package(BrainstemClient CONFIG REQUIRED)

    target_sources(lingtu_driver_io PRIVATE robots/doso/doso.cpp)
    target_include_directories(lingtu_driver_io PUBLIC
      "${CMAKE_CURRENT_SOURCE_DIR}/robots/doso")
    target_link_libraries(lingtu_driver_io PUBLIC brainstem::client)
  endif()

  if(LINGTU_DRIVER_BACKEND STREQUAL "go2")
    target_sources(lingtu_driver_io PRIVATE robots/unitree/go2/go2.cpp)
    target_link_libraries(lingtu_driver_io PUBLIC unitree_sdk2)
  endif()
  target_compile_options(lingtu_driver_io PRIVATE ${_LINGTU_DRIVER_WARNINGS})

  add_executable(lingtu_driver main.cpp status.cpp)
  target_link_libraries(lingtu_driver PRIVATE
    lingtu_driver_core
    lingtu_driver_io
    pthread)
  if(LINGTU_DRIVER_BACKEND STREQUAL "go2")
    set_target_properties(lingtu_driver PROPERTIES
      BUILD_RPATH "${LINGTU_UNITREE_SDK2_PREFIX}/lib"
      INSTALL_RPATH "${LINGTU_UNITREE_SDK2_PREFIX}/lib")
    target_link_options(lingtu_driver PRIVATE -Wl,--disable-new-dtags)
  endif()
  target_compile_options(lingtu_driver PRIVATE ${_LINGTU_DRIVER_WARNINGS})

  if(BUILD_TESTING AND NOT TARGET test_driver_status)
    add_executable(test_driver_status
      "${_LINGTU_DRIVER_TEST_DIR}/test_status.cpp"
      status.cpp)
    target_include_directories(test_driver_status PRIVATE
      "${CMAKE_CURRENT_SOURCE_DIR}"
      "${_LINGTU_ROOT}/src")
    target_link_libraries(test_driver_status PRIVATE lingtu_driver_core)
    target_compile_options(test_driver_status PRIVATE ${_LINGTU_DRIVER_WARNINGS})
    add_test(NAME test_driver_status COMMAND test_driver_status)
  endif()

  if(BUILD_TESTING AND LINGTU_DRIVER_BACKEND STREQUAL "doso")
    set(_BRAINSTEM_API_ROOT
      "${CMAKE_CURRENT_SOURCE_DIR}/robots/doso/protocol/brainstem_api")
    set(_PROTO_CMS_FILE "${_BRAINSTEM_API_ROOT}/brainstem_api/cms.proto")
    set(_PROTO_COMMON_FILE "${_BRAINSTEM_API_ROOT}/brainstem_api/common.proto")
    set(_PROTO_OUT "${CMAKE_CURRENT_BINARY_DIR}/test_proto")
    set(_PROTO_GENERATED
      "${_PROTO_OUT}/brainstem_api/cms.pb.cc"
      "${_PROTO_OUT}/brainstem_api/cms.pb.h"
      "${_PROTO_OUT}/brainstem_api/cms.grpc.pb.cc"
      "${_PROTO_OUT}/brainstem_api/cms.grpc.pb.h"
      "${_PROTO_OUT}/brainstem_api/common.pb.cc"
      "${_PROTO_OUT}/brainstem_api/common.pb.h")
    if(TARGET protobuf::protoc)
      set(_PROTOC "$<TARGET_FILE:protobuf::protoc>")
    else()
      set(_PROTOC "${Protobuf_PROTOC_EXECUTABLE}")
    endif()
    if(TARGET gRPC::grpc_cpp_plugin)
      set(_GRPC_CPP_PLUGIN "$<TARGET_FILE:gRPC::grpc_cpp_plugin>")
    else()
      find_program(_GRPC_CPP_PLUGIN NAMES grpc_cpp_plugin REQUIRED)
    endif()
    set(_PROTO_IMPORT_ARGS "--proto_path=${_BRAINSTEM_API_ROOT}")
    foreach(_PROTOBUF_INCLUDE IN LISTS Protobuf_INCLUDE_DIRS)
      list(APPEND _PROTO_IMPORT_ARGS "--proto_path=${_PROTOBUF_INCLUDE}")
    endforeach()
    add_custom_command(
      OUTPUT ${_PROTO_GENERATED}
      COMMAND "${CMAKE_COMMAND}" -E make_directory "${_PROTO_OUT}"
      COMMAND "${_PROTOC}"
        "--cpp_out=${_PROTO_OUT}"
        "--grpc_out=${_PROTO_OUT}"
        "--plugin=protoc-gen-grpc=${_GRPC_CPP_PLUGIN}"
        ${_PROTO_IMPORT_ARGS}
        "${_PROTO_COMMON_FILE}"
        "${_PROTO_CMS_FILE}"
      DEPENDS "${_PROTO_CMS_FILE}" "${_PROTO_COMMON_FILE}"
      VERBATIM)
    add_custom_target(lingtu_brainstem_test_protocol DEPENDS ${_PROTO_GENERATED})

    add_executable(test_driver_io
      "${_LINGTU_DRIVER_TEST_DIR}/test_doso_io.cpp"
      status.cpp)
    add_dependencies(test_driver_io lingtu_brainstem_test_protocol)
    target_include_directories(test_driver_io PRIVATE "${_PROTO_OUT}")
    target_link_libraries(test_driver_io PRIVATE
      lingtu_driver_core
      lingtu_driver_io
      pthread)
    target_compile_options(test_driver_io PRIVATE ${_LINGTU_DRIVER_WARNINGS})
    add_test(NAME test_driver_io COMMAND test_driver_io)
  endif()
endif()
