cmake_minimum_required(VERSION 3.16)
project(lingtu_camera_dds LANGUAGES C CXX)

if(NOT UNIX)
  message(FATAL_ERROR "lingtu_camera_dds is a Linux robot-side runtime")
endif()

find_package(CycloneDDS REQUIRED)
find_program(CYCLONEDDS_IDLC_EXECUTABLE NAMES idlc REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(_LINGTU_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../../../..")
set(_LINGTU_IDL "${_LINGTU_ROOT}/src/message/idl/messages.idl")
set(_LINGTU_IDL_HEADER "${CMAKE_CURRENT_BINARY_DIR}/messages.h")
set(_LINGTU_IDL_SOURCE "${CMAKE_CURRENT_BINARY_DIR}/messages.c")

add_custom_command(
  OUTPUT "${_LINGTU_IDL_HEADER}" "${_LINGTU_IDL_SOURCE}"
  COMMAND "${CYCLONEDDS_IDLC_EXECUTABLE}" -l c "${_LINGTU_IDL}"
  DEPENDS "${_LINGTU_IDL}"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  VERBATIM)

add_subdirectory(
  "${_LINGTU_ROOT}/src/message/cpp"
  "${CMAKE_CURRENT_BINARY_DIR}/message_cpp")

add_executable(lingtu_camera_dds
  camera_dds.cpp
  "${_LINGTU_IDL_SOURCE}")

target_include_directories(lingtu_camera_dds PRIVATE
  "${CMAKE_CURRENT_BINARY_DIR}"
  "${CMAKE_CURRENT_SOURCE_DIR}"
  "${_LINGTU_ROOT}/src")

target_link_libraries(lingtu_camera_dds PRIVATE
  CycloneDDS::ddsc
  lingtu_dds_contracts
  pthread)

target_compile_options(lingtu_camera_dds PRIVATE
  -Wall
  -Wextra
  -Wpedantic)

include(CTest)
if(BUILD_TESTING)
  find_package(Threads REQUIRED)
  add_executable(test_camera_record tests/test_camera_record.cpp)
  target_include_directories(test_camera_record PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}")
  target_compile_definitions(test_camera_record PRIVATE NDEBUG)
  target_compile_options(test_camera_record PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Werror)
  add_test(NAME lingtu_camera_record_contract COMMAND test_camera_record)

  add_executable(
    test_camera_record_deadline
    tests/test_camera_record_deadline.cpp)
  target_include_directories(test_camera_record_deadline PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}")
  target_compile_definitions(test_camera_record_deadline PRIVATE NDEBUG)
  target_compile_options(test_camera_record_deadline PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Werror)
  target_link_libraries(test_camera_record_deadline PRIVATE Threads::Threads)
  add_test(
    NAME lingtu_camera_record_absolute_deadline
    COMMAND test_camera_record_deadline)
endif()
