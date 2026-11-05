cmake_minimum_required(VERSION 3.16)

foreach(DEPRECATED_OPTION IN ITEMS
    LINGTU_SLAM_BUILD_CPP_DDS_RUNTIME
    LINGTU_SLAM_BUILD_CYCLONE_DDS_RUNTIME
    LINGTU_SLAM_WITH_FASTLIO2
    LINGTU_SLAM_FASTLIO2_BACKEND)
  if(DEFINED ${DEPRECATED_OPTION})
    if(DEPRECATED_OPTION STREQUAL "LINGTU_SLAM_WITH_FASTLIO2" OR
       DEPRECATED_OPTION STREQUAL "LINGTU_SLAM_FASTLIO2_BACKEND")
      set(REPLACEMENT_OPTION LINGTU_SLAM_LTLIO_BACKEND)
    else()
      set(REPLACEMENT_OPTION LINGTU_SLAM_BUILD_DDS_RUNTIME)
    endif()
    message(FATAL_ERROR
      "${DEPRECATED_OPTION} was removed; use ${REPLACEMENT_OPTION} instead.")
  endif()
endforeach()

option(LINGTU_SLAM_BUILD_TESTS "Build native SLAM contract tests" ON)
option(LINGTU_SLAM_LTLIO_BACKEND "Build the native LTLIO algorithm backend" OFF)
option(LINGTU_SLAM_BUILD_DDS_RUNTIME "Build ROS-free CycloneDDS C++ sensor-to-SLAM runtime" OFF)
if(LINGTU_SLAM_BUILD_DDS_RUNTIME AND NOT LINGTU_SLAM_LTLIO_BACKEND)
  message(FATAL_ERROR
    "LINGTU_SLAM_BUILD_DDS_RUNTIME=ON requires "
    "LINGTU_SLAM_LTLIO_BACKEND=ON; refusing to build production slamd "
    "against the contract-only backend.")
endif()

project(messages_contract LANGUAGES C CXX)

include("${CMAKE_CURRENT_LIST_DIR}/../../../../scripts/build/cmake/settings.cmake")
if(MSVC)
  add_compile_options(/utf-8)
endif()
include(GNUInstallDirs)

add_library(messages_contract STATIC
  slam.cpp
  ltlio.cpp
)
target_compile_features(messages_contract PUBLIC cxx_std_17)
target_link_libraries(messages_contract PUBLIC "$<BUILD_INTERFACE:lingtu_settings>")
target_include_directories(messages_contract PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
set_target_properties(messages_contract PROPERTIES POSITION_INDEPENDENT_CODE ON)

if(LINGTU_SLAM_LTLIO_BACKEND)
  set(LTLIO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../ltlio")
  set(RELOCALIZATION_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../relocalization")
  find_package(PCL REQUIRED COMPONENTS common io filters kdtree registration)
  find_package(yaml-cpp REQUIRED)
  if(TARGET yaml-cpp::yaml-cpp)
    set(LINGTU_YAML_CPP_TARGET yaml-cpp::yaml-cpp)
  elseif(TARGET yaml-cpp)
    set(LINGTU_YAML_CPP_TARGET yaml-cpp)
  else()
    message(FATAL_ERROR "yaml-cpp package did not export a supported CMake target")
  endif()
  if(LINGTU_SLAM_BUILD_TESTS)
    set(LTLIO_ENABLE_INTERNAL_TEST_HOOKS ON)
  endif()
  add_subdirectory("${LTLIO_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/ltlio")
  add_subdirectory("${RELOCALIZATION_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/relocalization")
  if(NOT TARGET lingtu_online_mapping)
    add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../../online_mapping" "${CMAKE_CURRENT_BINARY_DIR}/online_mapping")
  endif()

  target_sources(messages_contract PRIVATE
    map_icp.cpp
    semantic_map_client.cpp
    native_relocalizer.cpp
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../maps/cpp/semantic/store.cpp"
  )
  target_compile_definitions(messages_contract PRIVATE
    LINGTU_HAS_LTLIO_BACKEND=1)
  target_include_directories(messages_contract PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../maps/include")
  target_link_libraries(messages_contract
    PRIVATE relocalization lingtu_online_mapping
    PUBLIC ltlio_core ${LINGTU_YAML_CPP_TARGET})

  add_executable(ltlio_replay "${CMAKE_CURRENT_SOURCE_DIR}/../../../../tools/diagnostics/ltlio_replay.cpp")
  target_link_libraries(ltlio_replay PRIVATE messages_contract)
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
  add_executable(test_map_frames "${LINGTU_SLAM_TESTS_DIR}/test_map_frames.cpp")
  target_link_libraries(test_map_frames PRIVATE messages_contract)
  add_test(NAME messages_map_frames COMMAND test_map_frames)
  add_executable(test_travel_distance "${LINGTU_SLAM_TESTS_DIR}/test_distance.cpp")
  target_link_libraries(test_travel_distance PRIVATE messages_contract)
  add_test(NAME messages_travel_distance COMMAND test_travel_distance)
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
  add_executable(test_manual_relocalization
    "${LINGTU_SLAM_TESTS_DIR}/test_manual_relocalization.cpp")
  target_link_libraries(test_manual_relocalization PRIVATE messages_contract)
  add_test(NAME messages_manual_relocalization COMMAND test_manual_relocalization)
  add_executable(test_imu_frame_contract
    "${LINGTU_SLAM_TESTS_DIR}/test_imu_frame_contract.cpp")
  target_link_libraries(test_imu_frame_contract PRIVATE messages_contract)
  add_test(NAME messages_imu_frame_contract COMMAND test_imu_frame_contract)
  if(LINGTU_SLAM_LTLIO_BACKEND)
    add_executable(test_elevator_native
      "${LINGTU_SLAM_TESTS_DIR}/test_elevator_native.cpp")
    target_link_libraries(test_elevator_native PRIVATE messages_contract)
    add_test(NAME messages_elevator_native COMMAND test_elevator_native)
    set_tests_properties(messages_elevator_native PROPERTIES TIMEOUT 60)
    add_executable(test_ltlio_elevator_transitions
      "${LINGTU_SLAM_TESTS_DIR}/test_ltlio_elevator_transitions.cpp")
    target_link_libraries(test_ltlio_elevator_transitions PRIVATE ltlio_core)
    add_test(NAME messages_ltlio_elevator_transitions COMMAND test_ltlio_elevator_transitions)
    add_executable(test_ltlio_imu_time
      "${LINGTU_SLAM_TESTS_DIR}/test_ltlio_imu_time.cpp")
    target_link_libraries(test_ltlio_imu_time PRIVATE ltlio_core)
    add_test(NAME messages_ltlio_imu_time COMMAND test_ltlio_imu_time)
    add_executable(test_ltlio_stair_detection
      "${LINGTU_SLAM_TESTS_DIR}/test_ltlio_stair_detection.cpp")
    target_link_libraries(test_ltlio_stair_detection PRIVATE ltlio_core)
    add_test(NAME messages_ltlio_stair_detection COMMAND test_ltlio_stair_detection)
    add_executable(test_ltlio_stop_causality
      "${LINGTU_SLAM_TESTS_DIR}/test_ltlio_stop_causality.cpp")
    target_link_libraries(test_ltlio_stop_causality PRIVATE ltlio_core)
    add_test(NAME messages_ltlio_stop_causality COMMAND test_ltlio_stop_causality)
    add_executable(test_ltlio_frontend_recovery
      "${LINGTU_SLAM_TESTS_DIR}/test_ltlio_frontend_recovery.cpp")
    target_link_libraries(test_ltlio_frontend_recovery PRIVATE ltlio_core)
    add_test(NAME messages_ltlio_frontend_recovery COMMAND test_ltlio_frontend_recovery)
    add_executable(test_ltlio_geometry "${LINGTU_SLAM_TESTS_DIR}/test_ltlio_geometry.cpp")
    target_include_directories(test_ltlio_geometry PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
    target_link_libraries(test_ltlio_geometry PRIVATE ltlio_core)
    add_test(NAME messages_ltlio_geometry COMMAND test_ltlio_geometry)
    add_executable(test_ltlio_probability "${LINGTU_SLAM_TESTS_DIR}/test_ltlio_probability.cpp")
    target_link_libraries(test_ltlio_probability PRIVATE ltlio_core)
    target_compile_definitions(test_ltlio_probability PRIVATE
      $<TARGET_PROPERTY:ltlio_core,COMPILE_DEFINITIONS>)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
      target_compile_options(test_ltlio_probability PRIVATE -O1)
    endif()
    add_test(NAME messages_ltlio_probability COMMAND test_ltlio_probability)
    add_executable(test_ltlio_isotropic_calibration "${LINGTU_SLAM_TESTS_DIR}/test_ltlio_isotropic_calibration.cpp")
    target_link_libraries(test_ltlio_isotropic_calibration PRIVATE ltlio_core)
    target_compile_definitions(test_ltlio_isotropic_calibration PRIVATE
      $<TARGET_PROPERTY:ltlio_core,COMPILE_DEFINITIONS>)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
      target_compile_options(test_ltlio_isotropic_calibration PRIVATE -O1)
    endif()
    add_test(NAME messages_ltlio_isotropic_calibration COMMAND test_ltlio_isotropic_calibration)
    foreach(suite hybrid scale_sampling)
      add_executable(test_ltlio_${suite} "${LINGTU_SLAM_TESTS_DIR}/test_ltlio_${suite}.cpp")
      target_link_libraries(test_ltlio_${suite} PRIVATE ltlio_core)
      target_compile_definitions(test_ltlio_${suite} PRIVATE
        $<TARGET_PROPERTY:ltlio_core,COMPILE_DEFINITIONS>)
      if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(test_ltlio_${suite} PRIVATE -O1)
      endif()
      add_test(NAME messages_ltlio_${suite} COMMAND test_ltlio_${suite})
    endforeach()
    set_tests_properties(messages_ltlio_elevator_transitions PROPERTIES TIMEOUT 60)
    add_executable(test_ltlio_transport_motion
      "${LINGTU_SLAM_TESTS_DIR}/test_ltlio_transport_motion.cpp")
    target_link_libraries(test_ltlio_transport_motion PRIVATE messages_contract)
    add_test(NAME messages_ltlio_transport_motion COMMAND test_ltlio_transport_motion)
    set_tests_properties(messages_ltlio_transport_motion PROPERTIES TIMEOUT 60)
    add_executable(test_ltlio_resources "${LINGTU_SLAM_TESTS_DIR}/test_ltlio_resources.cpp")
    target_link_libraries(test_ltlio_resources PRIVATE ltlio_core)
    add_test(NAME messages_ltlio_resources COMMAND test_ltlio_resources)
    add_executable(test_ltlio_profiles "${LINGTU_SLAM_TESTS_DIR}/test_ltlio_profiles.cpp")
    target_link_libraries(test_ltlio_profiles PRIVATE messages_contract)
    add_test(NAME messages_ltlio_profiles COMMAND test_ltlio_profiles)
    add_executable(test_semantic_map_loading
      "${LINGTU_SLAM_TESTS_DIR}/test_semantic_map_loading.cpp")
    target_include_directories(test_semantic_map_loading PRIVATE
      "${CMAKE_CURRENT_SOURCE_DIR}/../../../maps/include")
    target_link_libraries(test_semantic_map_loading PRIVATE messages_contract)
    add_test(NAME messages_semantic_map_loading COMMAND test_semantic_map_loading)
    add_test(NAME messages_ikd_tree_concurrency COMMAND test_ikd_tree_concurrency)
    set_tests_properties(messages_ikd_tree_concurrency PROPERTIES TIMEOUT 30)
    add_executable(test_ltlio_mock_flow
      "${LINGTU_SLAM_TESTS_DIR}/test_ltlio_mock_flow.cpp")
    target_link_libraries(test_ltlio_mock_flow PRIVATE messages_contract)
    add_test(NAME messages_ltlio_mock_flow COMMAND test_ltlio_mock_flow)
    add_test(NAME messages_ltlio_worker_capacity COMMAND test_ltlio_mock_flow --worker-capacity)
    set_tests_properties(messages_ltlio_worker_capacity PROPERTIES TIMEOUT 120)
    add_executable(test_map_geometry_handoff "${LINGTU_SLAM_TESTS_DIR}/test_map_geometry_handoff.cpp")
    target_link_libraries(test_map_geometry_handoff PRIVATE messages_contract)
    add_test(NAME messages_map_geometry_handoff COMMAND test_map_geometry_handoff)
    set_tests_properties(messages_map_geometry_handoff PROPERTIES TIMEOUT 120)
    add_executable(test_elevator_persistence "${LINGTU_SLAM_TESTS_DIR}/test_elevator_persistence.cpp")
    target_link_libraries(test_elevator_persistence PRIVATE messages_contract)
    add_test(NAME messages_elevator_persistence COMMAND test_elevator_persistence)
    add_executable(test_initial_body_origin
      "${LINGTU_SLAM_TESTS_DIR}/test_initial_body_origin.cpp")
    target_link_libraries(test_initial_body_origin PRIVATE messages_contract)
    add_test(NAME messages_initial_body_origin COMMAND test_initial_body_origin)
    add_executable(test_icp_diagnostics
      "${LINGTU_SLAM_TESTS_DIR}/test_icp_diagnostics.cpp")
    target_link_libraries(test_icp_diagnostics PRIVATE relocalization)
    add_test(NAME messages_icp_diagnostics COMMAND test_icp_diagnostics)
    add_executable(test_map_icp_seed
      "${LINGTU_SLAM_TESTS_DIR}/test_map_icp_seed.cpp")
    target_link_libraries(test_map_icp_seed PRIVATE messages_contract relocalization)
    add_test(NAME messages_map_icp_seed COMMAND test_map_icp_seed)
    add_executable(test_imu_acceleration_scale
      "${LINGTU_SLAM_TESTS_DIR}/test_imu_acceleration_scale.cpp")
    target_link_libraries(test_imu_acceleration_scale PRIVATE
      ltlio_core
      ${LINGTU_YAML_CPP_TARGET})
    add_test(
      NAME messages_imu_acceleration_scale
      COMMAND test_imu_acceleration_scale)
  endif()
endif()

if(LINGTU_SLAM_BUILD_DDS_RUNTIME)
  find_package(iceoryx_binding_c QUIET)
  find_package(CycloneDDS REQUIRED)

  include("${CMAKE_CURRENT_SOURCE_DIR}/../../../../cmake/LingTuDDS.cmake")

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

  if(LINGTU_SLAM_BUILD_TESTS)
    add_executable(test_tf_publication
      "${LINGTU_SLAM_TESTS_DIR}/test_tf_publication.cpp")
    target_compile_features(test_tf_publication PRIVATE cxx_std_17)
    target_include_directories(test_tf_publication PRIVATE
      "${CMAKE_CURRENT_SOURCE_DIR}")
    target_link_libraries(test_tf_publication PRIVATE
      messages_contract
      messages_cyclone_idl
      lingtu_dds_contracts
      CycloneDDS::ddsc)
    add_test(NAME messages_tf_publication COMMAND test_tf_publication)
    set_tests_properties(messages_tf_publication PROPERTIES TIMEOUT 15)
    add_executable(test_frame_export "${LINGTU_SLAM_TESTS_DIR}/test_frame_export.cpp")
    target_include_directories(test_frame_export PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
    target_link_libraries(test_frame_export PRIVATE
      messages_contract messages_cyclone_idl lingtu_dds_contracts CycloneDDS::ddsc)
    add_test(NAME messages_frame_export COMMAND test_frame_export)
    add_executable(test_odometry_publication
      "${LINGTU_SLAM_TESTS_DIR}/test_odometry_publication.cpp")
    target_compile_features(test_odometry_publication PRIVATE cxx_std_17)
    target_include_directories(test_odometry_publication PRIVATE
      "${CMAKE_CURRENT_SOURCE_DIR}")
    target_link_libraries(test_odometry_publication PRIVATE
      messages_contract
      messages_cyclone_idl
      lingtu_dds_contracts
      CycloneDDS::ddsc)
    add_test(NAME messages_odometry_publication COMMAND test_odometry_publication)

    add_executable(test_track_seed
      "${LINGTU_SLAM_TESTS_DIR}/test_track_seed.cpp")
    target_compile_features(test_track_seed PRIVATE cxx_std_17)
    target_include_directories(test_track_seed PRIVATE
      "${CMAKE_CURRENT_SOURCE_DIR}")
    target_link_libraries(test_track_seed PRIVATE
      messages_contract
      messages_cyclone_idl
      lingtu_dds_contracts
      CycloneDDS::ddsc)
    add_test(NAME messages_track_seed COMMAND test_track_seed)
  endif()

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
  set_target_properties(slamd PROPERTIES
    INSTALL_RPATH "$ORIGIN/../${CMAKE_INSTALL_LIBDIR}")
  if(CPU_BBS3D_LIBRARY)
    install(FILES "${CPU_BBS3D_LIBRARY}"
      DESTINATION ${CMAKE_INSTALL_LIBDIR}
      COMPONENT lingtu_runtime)
  endif()
  if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    foreach(_pcl_runtime IN LISTS PCL_LIBRARIES)
      if(_pcl_runtime MATCHES "^pcl_" AND TARGET ${_pcl_runtime})
        get_target_property(_pcl_runtime_type ${_pcl_runtime} TYPE)
        if(_pcl_runtime_type STREQUAL "SHARED_LIBRARY")
          install(CODE
            "file(INSTALL DESTINATION \"\${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}\"
              TYPE SHARED_LIBRARY FOLLOW_SYMLINK_CHAIN
              FILES \"$<TARGET_FILE:${_pcl_runtime}>\")"
            COMPONENT lingtu_runtime)
        endif()
      endif()
    endforeach()
    # PCL's IO dependency is not exported as a standalone imported target.
    unset(_pcl_io_ply_runtime CACHE)
    find_library(_pcl_io_ply_runtime NAMES pcl_io_ply
      PATHS ${PCL_LIBRARY_DIRS} NO_DEFAULT_PATH)
    if(NOT _pcl_io_ply_runtime)
      message(FATAL_ERROR "PCL IO runtime is missing pcl_io_ply")
    endif()
    install(CODE
      "file(INSTALL DESTINATION \"\${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}\"
        TYPE SHARED_LIBRARY FOLLOW_SYMLINK_CHAIN
        FILES \"${_pcl_io_ply_runtime}\")"
      COMPONENT lingtu_runtime)
  endif()
endif()
