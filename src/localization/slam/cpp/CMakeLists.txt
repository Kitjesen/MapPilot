cmake_minimum_required(VERSION 3.16)

foreach(DEPRECATED_OPTION IN ITEMS
    LINGTU_SLAM_BUILD_CPP_DDS_RUNTIME
    LINGTU_SLAM_BUILD_CYCLONE_DDS_RUNTIME
    LINGTU_SLAM_WITH_FASTLIO2)
  if(DEFINED ${DEPRECATED_OPTION})
    if(DEPRECATED_OPTION STREQUAL "LINGTU_SLAM_WITH_FASTLIO2")
      set(REPLACEMENT_OPTION LINGTU_SLAM_FASTLIO2_BACKEND)
    else()
      set(REPLACEMENT_OPTION LINGTU_SLAM_BUILD_DDS_RUNTIME)
    endif()
    message(FATAL_ERROR
      "${DEPRECATED_OPTION} was removed; use ${REPLACEMENT_OPTION} instead.")
  endif()
endforeach()

project(messages_contract LANGUAGES C CXX)
include(GNUInstallDirs)

option(LINGTU_SLAM_BUILD_TESTS "Build native SLAM contract tests" ON)
option(LINGTU_SLAM_FASTLIO2_BACKEND "Build the real Fast-LIO2 algorithm backend" OFF)
option(LINGTU_SLAM_BUILD_DDS_RUNTIME "Build ROS-free CycloneDDS C++ sensor-to-SLAM runtime" OFF)
if(LINGTU_SLAM_BUILD_DDS_RUNTIME AND NOT LINGTU_SLAM_FASTLIO2_BACKEND)
  message(FATAL_ERROR
    "LINGTU_SLAM_BUILD_DDS_RUNTIME=ON requires "
    "LINGTU_SLAM_FASTLIO2_BACKEND=ON; refusing to build production slamd "
    "against the contract-only backend.")
endif()
add_library(messages_contract STATIC
  slam.cpp
  fastlio.cpp
)
target_compile_features(messages_contract PUBLIC cxx_std_17)
target_include_directories(messages_contract PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
set_target_properties(messages_contract PROPERTIES POSITION_INDEPENDENT_CODE ON)

if(LINGTU_SLAM_FASTLIO2_BACKEND)
  set(FASTLIO2_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../fastlio2")
  set(LOCALIZER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../localizer")
  find_package(yaml-cpp REQUIRED)
  if(TARGET yaml-cpp::yaml-cpp)
    set(LINGTU_YAML_CPP_TARGET yaml-cpp::yaml-cpp)
  elseif(TARGET yaml-cpp)
    set(LINGTU_YAML_CPP_TARGET yaml-cpp)
  else()
    message(FATAL_ERROR "yaml-cpp package did not export a supported CMake target")
  endif()
  if(LINGTU_SLAM_BUILD_TESTS)
    set(FASTLIO2_ENABLE_INTERNAL_TEST_HOOKS ON)
  endif()
  add_subdirectory("${FASTLIO2_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/fastlio2")
  add_subdirectory("${LOCALIZER_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/localizer")

  target_sources(messages_contract PRIVATE
    map_icp.cpp
    semantic_map_client.cpp
    native_relocalizer.cpp
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../maps/cpp/semantic_map_persistence.cpp"
  )
  target_compile_definitions(messages_contract PRIVATE
    LINGTU_HAS_FASTLIO2_BACKEND=1)
  target_include_directories(messages_contract PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../maps/include")
  target_link_libraries(messages_contract
    PRIVATE localizer
    PUBLIC fastlio2_core ${LINGTU_YAML_CPP_TARGET})
endif()

if(MSVC)
  target_compile_options(messages_contract PRIVATE /W4)
else()
  target_compile_options(messages_contract PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(LINGTU_SLAM_BUILD_TESTS)
  enable_testing()
  set(LINGTU_SLAM_TESTS_DIR
      "${CMAKE_CURRENT_SOURCE_DIR}/../../../../tests/localization/cpp/slam")
  add_executable(test_slam_contract "${LINGTU_SLAM_TESTS_DIR}/test_slam.cpp")
  target_link_libraries(test_slam_contract PRIVATE messages_contract)
  add_test(NAME messages_contract COMMAND test_slam_contract)
  add_executable(test_relocalization_gate
    "${LINGTU_SLAM_TESTS_DIR}/test_relocalization_gate.cpp")
  target_link_libraries(test_relocalization_gate PRIVATE messages_contract)
  add_test(
    NAME messages_relocalization_gate
    COMMAND test_relocalization_gate)
  add_executable(test_map_tracking_health
    "${LINGTU_SLAM_TESTS_DIR}/test_map_tracking_health.cpp")
  target_link_libraries(test_map_tracking_health PRIVATE messages_contract)
  add_test(NAME messages_map_tracking_health COMMAND test_map_tracking_health)
  add_executable(test_imu_frame_contract
    "${LINGTU_SLAM_TESTS_DIR}/test_imu_frame_contract.cpp")
  target_link_libraries(test_imu_frame_contract PRIVATE messages_contract)
  add_test(NAME messages_imu_frame_contract COMMAND test_imu_frame_contract)
  if(LINGTU_SLAM_FASTLIO2_BACKEND)
    add_executable(test_semantic_map_loading
      "${LINGTU_SLAM_TESTS_DIR}/test_semantic_map_loading.cpp")
    target_include_directories(test_semantic_map_loading PRIVATE
      "${CMAKE_CURRENT_SOURCE_DIR}/../../../maps/include")
    target_link_libraries(test_semantic_map_loading PRIVATE messages_contract)
    add_test(NAME messages_semantic_map_loading COMMAND test_semantic_map_loading)
    add_executable(test_ikd_tree_concurrency
      "${LINGTU_SLAM_TESTS_DIR}/test_ikd_tree_concurrency.cpp")
    target_link_libraries(test_ikd_tree_concurrency PRIVATE fastlio2_core)
    add_test(NAME messages_ikd_tree_concurrency COMMAND test_ikd_tree_concurrency)
    set_tests_properties(messages_ikd_tree_concurrency PROPERTIES TIMEOUT 30)
    add_executable(test_ieskf_initialization
      "${LINGTU_SLAM_TESTS_DIR}/test_ieskf_initialization.cpp")
    target_include_directories(test_ieskf_initialization PRIVATE
      "${FASTLIO2_DIR}/src"
      "${FASTLIO2_DIR}/src/map_builder")
    target_link_libraries(test_ieskf_initialization PRIVATE messages_contract)
    add_test(NAME messages_ieskf_initialization COMMAND test_ieskf_initialization)
    add_executable(test_ieskf_update_diagnostics
      "${LINGTU_SLAM_TESTS_DIR}/test_ieskf_update_diagnostics.cpp")
    target_include_directories(test_ieskf_update_diagnostics PRIVATE
      "${FASTLIO2_DIR}/src"
      "${FASTLIO2_DIR}/src/map_builder")
    target_link_libraries(test_ieskf_update_diagnostics PRIVATE messages_contract)
    add_test(
      NAME messages_ieskf_update_diagnostics
      COMMAND test_ieskf_update_diagnostics)
    add_executable(test_fastlio2_mock_flow
      "${LINGTU_SLAM_TESTS_DIR}/test_fastlio2_mock_flow.cpp")
    target_link_libraries(test_fastlio2_mock_flow PRIVATE messages_contract)
    add_test(NAME messages_fastlio2_mock_flow COMMAND test_fastlio2_mock_flow)
    add_executable(test_initial_body_origin
      "${LINGTU_SLAM_TESTS_DIR}/test_initial_body_origin.cpp")
    target_link_libraries(test_initial_body_origin PRIVATE messages_contract)
    add_test(NAME messages_initial_body_origin COMMAND test_initial_body_origin)
    add_executable(test_icp_diagnostics
      "${LINGTU_SLAM_TESTS_DIR}/test_icp_diagnostics.cpp")
    target_link_libraries(test_icp_diagnostics PRIVATE localizer)
    add_test(NAME messages_icp_diagnostics COMMAND test_icp_diagnostics)
    add_executable(test_imu_acceleration_scale
      "${LINGTU_SLAM_TESTS_DIR}/test_imu_acceleration_scale.cpp")
    target_link_libraries(test_imu_acceleration_scale PRIVATE
      fastlio2_core
      ${LINGTU_YAML_CPP_TARGET})
    add_test(
      NAME messages_imu_acceleration_scale
      COMMAND test_imu_acceleration_scale)
  endif()
endif()

if(LINGTU_SLAM_BUILD_DDS_RUNTIME)
  find_package(iceoryx_binding_c QUIET)
  find_package(CycloneDDS REQUIRED)

  add_subdirectory(
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../message/cpp"
    "${CMAKE_CURRENT_BINARY_DIR}/message_cpp")

  set(_LINGTU_SLAM_IDL "${CMAKE_CURRENT_SOURCE_DIR}/../../../message/idl/messages.idl")
  lingtu_add_dds_c_messages(messages_cyclone_idl "${_LINGTU_SLAM_IDL}")

  add_executable(slamd cyclone_runtime.cpp)
  target_compile_features(slamd PRIVATE cxx_std_17)
  target_include_directories(slamd PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../../..")
  target_link_libraries(slamd
    messages_contract
    messages_cyclone_idl
    lingtu_dds_contracts
    CycloneDDS::ddsc)

  add_executable(slamctl slam_control.cpp)
  target_compile_features(slamctl PRIVATE cxx_std_17)
  target_include_directories(slamctl PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../../..")
  target_link_libraries(slamctl
    messages_cyclone_idl
    lingtu_dds_contracts
    CycloneDDS::ddsc)

  install(TARGETS slamd slamctl
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
      COMPONENT lingtu_runtime)
endif()
