cmake_minimum_required(VERSION 3.16)
project(lingtu_driver LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(CTest)

add_library(lingtu_driver_core STATIC
  command_freshness_gate.cpp
  core.cpp
  config.cpp)
target_include_directories(lingtu_driver_core PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_options(lingtu_driver_core PRIVATE -Wall -Wextra -Wpedantic)

if(BUILD_TESTING)
  add_executable(test_driver_core test_core.cpp)
  target_link_libraries(test_driver_core PRIVATE lingtu_driver_core)
  target_compile_options(test_driver_core PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_driver_core COMMAND test_driver_core)
endif()

if(UNIX)
  set(_DRIVER_RUNTIME_DEFAULT ON)
else()
  set(_DRIVER_RUNTIME_DEFAULT OFF)
endif()
option(LINGTU_DRIVER_BUILD_RUNTIME "Build the CycloneDDS to Brainstem driver" ${_DRIVER_RUNTIME_DEFAULT})

if(LINGTU_DRIVER_BUILD_RUNTIME)
  if(NOT UNIX)
    message(FATAL_ERROR "lingtu_driver runtime is supported on Linux; build only tests on this host")
  endif()

  find_package(CycloneDDS REQUIRED)
  find_package(Protobuf REQUIRED)
  find_package(gRPC CONFIG REQUIRED)
  find_program(CYCLONEDDS_IDLC_EXECUTABLE NAMES idlc REQUIRED)

  get_filename_component(_LINGTU_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../../../.." ABSOLUTE)
  set(_LINGTU_IDL "${_LINGTU_ROOT}/src/message/idl/lingtu_slam.idl")
  set(_LINGTU_IDL_HEADER "${CMAKE_CURRENT_BINARY_DIR}/lingtu_slam.h")
  set(_LINGTU_IDL_SOURCE "${CMAKE_CURRENT_BINARY_DIR}/lingtu_slam.c")
  add_custom_command(
    OUTPUT "${_LINGTU_IDL_HEADER}" "${_LINGTU_IDL_SOURCE}"
    COMMAND "${CYCLONEDDS_IDLC_EXECUTABLE}" -l c "${_LINGTU_IDL}"
    DEPENDS "${_LINGTU_IDL}"
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
    VERBATIM)

  set(_PROTO_FILE "${CMAKE_CURRENT_SOURCE_DIR}/brainstem.proto")
  set(_PROTO_OUT "${CMAKE_CURRENT_BINARY_DIR}/proto")
  file(MAKE_DIRECTORY "${_PROTO_OUT}")
  set(_PROTO_PB_CC "${_PROTO_OUT}/brainstem.pb.cc")
  set(_PROTO_PB_H "${_PROTO_OUT}/brainstem.pb.h")
  set(_PROTO_GRPC_CC "${_PROTO_OUT}/brainstem.grpc.pb.cc")
  set(_PROTO_GRPC_H "${_PROTO_OUT}/brainstem.grpc.pb.h")

  if(TARGET protobuf::protoc)
    set(_PROTOC "$<TARGET_FILE:protobuf::protoc>")
  elseif(Protobuf_PROTOC_EXECUTABLE)
    set(_PROTOC "${Protobuf_PROTOC_EXECUTABLE}")
  else()
    message(FATAL_ERROR "protobuf compiler was not found")
  endif()
  if(TARGET gRPC::grpc_cpp_plugin)
    set(_GRPC_CPP_PLUGIN "$<TARGET_FILE:gRPC::grpc_cpp_plugin>")
  else()
    find_program(_GRPC_CPP_PLUGIN NAMES grpc_cpp_plugin REQUIRED)
  endif()

  set(_PROTO_IMPORT_ARGS "--proto_path=${CMAKE_CURRENT_SOURCE_DIR}")
  foreach(_PROTOBUF_INCLUDE IN LISTS Protobuf_INCLUDE_DIRS)
    list(APPEND _PROTO_IMPORT_ARGS "--proto_path=${_PROTOBUF_INCLUDE}")
  endforeach()
  add_custom_command(
    OUTPUT "${_PROTO_PB_CC}" "${_PROTO_PB_H}" "${_PROTO_GRPC_CC}" "${_PROTO_GRPC_H}"
    COMMAND "${_PROTOC}"
      "--cpp_out=${_PROTO_OUT}"
      "--grpc_out=${_PROTO_OUT}"
      "--plugin=protoc-gen-grpc=${_GRPC_CPP_PLUGIN}"
      ${_PROTO_IMPORT_ARGS}
      "${_PROTO_FILE}"
    DEPENDS "${_PROTO_FILE}"
    VERBATIM)

  add_subdirectory(
    "${_LINGTU_ROOT}/src/message/cpp"
    "${CMAKE_CURRENT_BINARY_DIR}/message_cpp")

  add_library(lingtu_driver_io STATIC
    brainstem.cpp
    dds.cpp
    "${_PROTO_PB_CC}"
    "${_PROTO_GRPC_CC}"
    "${_LINGTU_IDL_SOURCE}")
  target_include_directories(lingtu_driver_io PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_PROTO_OUT}"
    "${_LINGTU_ROOT}/src")
  target_link_libraries(lingtu_driver_io PUBLIC
    lingtu_dds_qos_profiles
    CycloneDDS::ddsc
    gRPC::grpc++
    protobuf::libprotobuf)
  target_compile_options(lingtu_driver_io PRIVATE -Wall -Wextra -Wpedantic)

  add_executable(lingtu_driver main.cpp status.cpp)
  target_link_libraries(lingtu_driver PRIVATE
    lingtu_driver_core
    lingtu_driver_io
    pthread)
  target_compile_options(lingtu_driver PRIVATE -Wall -Wextra -Wpedantic)

  if(BUILD_TESTING)
    add_executable(test_driver_io test_io.cpp status.cpp)
    target_link_libraries(test_driver_io PRIVATE
      lingtu_driver_core
      lingtu_driver_io
      pthread)
    target_compile_options(test_driver_io PRIVATE -Wall -Wextra -Wpedantic)
    add_test(NAME test_driver_io COMMAND test_driver_io)
  endif()
endif()
