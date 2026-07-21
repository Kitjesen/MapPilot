cmake_minimum_required(VERSION 3.16)

project(lingtu_maps LANGUAGES CXX)

find_package(Threads REQUIRED)

option(LINGTU_MAPS_ENABLE_NATIVE_OCTOMAP "Enable embedded OctoMap artifact builder when OctoMap is available" ON)

if(LINGTU_MAPS_ENABLE_NATIVE_OCTOMAP)
  find_package(octomap QUIET)
endif()

add_library(lingtu_maps_contract INTERFACE)
target_include_directories(
  lingtu_maps_contract
  INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_compile_features(lingtu_maps_contract INTERFACE cxx_std_17)

add_library(lingtu_maps STATIC
  cpp/active_slots.cpp
  cpp/artifact_jobs.cpp
  cpp/block_grid.cpp
  cpp/hash.cpp
  cpp/health.cpp
  cpp/lock.cpp
  cpp/maintenance.cpp
  cpp/map_graph.cpp
  cpp/scene.cpp
  cpp/build/grid_artifacts.cpp
  cpp/build/occupancy_snapshot.cpp
  cpp/build/pcd.cpp
  cpp/build/pipeline.cpp
  cpp/build/process.cpp
  cpp/semantic_map_persistence.cpp
  cpp/save.cpp
  cpp/service.cpp
  cpp/store.cpp
  cpp/version.cpp
  cpp/layers/rolling_occupancy.cpp
  cpp/layers/exploration_projection.cpp
  cpp/layers/semantic_occupancy.cpp
  cpp/layers/voxel.cpp
)
target_link_libraries(lingtu_maps PUBLIC lingtu_maps_contract Threads::Threads)
if(octomap_FOUND)
  target_compile_definitions(lingtu_maps PRIVATE LINGTU_MAPS_HAS_OCTOMAP=1)
  if(DEFINED OCTOMAP_INCLUDE_DIRS)
    target_include_directories(lingtu_maps PRIVATE ${OCTOMAP_INCLUDE_DIRS})
  endif()
  if(DEFINED OCTOMAP_LIBRARIES)
    target_link_libraries(lingtu_maps PRIVATE ${OCTOMAP_LIBRARIES})
  else()
    target_link_libraries(lingtu_maps PRIVATE octomap)
  endif()
endif()
set_target_properties(lingtu_maps PROPERTIES
  OUTPUT_NAME "lingtu_maps_core"
  POSITION_INDEPENDENT_CODE ON
)

add_library(lingtu_maps_c_api SHARED
  cpp/grid_layers_c_api.cpp
  cpp/pcd_c_api.cpp
  cpp/semantic_c_api.cpp
  cpp/service_c_api.cpp
  cpp/store_c_api.cpp
  cpp/voxel_c_api.cpp
)
target_link_libraries(lingtu_maps_c_api PRIVATE lingtu_maps)
target_compile_definitions(lingtu_maps_c_api PRIVATE LINGTU_MAPS_C_API_BUILD)
set_target_properties(lingtu_maps_c_api PROPERTIES
  OUTPUT_NAME "lingtu_maps"
)

option(LINGTU_MAPS_BUILD_TESTS "Build LingTu maps contract tests" OFF)

if(LINGTU_MAPS_BUILD_TESTS)
  enable_testing()

  add_executable(lingtu_maps_header_smoke tests/cpp/header_smoke.cpp)
  target_link_libraries(lingtu_maps_header_smoke PRIVATE lingtu_maps)
  add_test(NAME lingtu_maps_header_smoke COMMAND lingtu_maps_header_smoke)

  add_executable(lingtu_maps_grid_layers_test tests/cpp/grid_layers_test.cpp)
  target_link_libraries(lingtu_maps_grid_layers_test PRIVATE lingtu_maps)
  add_test(NAME lingtu_maps_grid_layers_test COMMAND lingtu_maps_grid_layers_test)

  add_executable(lingtu_maps_voxel_layer_test tests/cpp/voxel_layer_test.cpp)
  target_link_libraries(lingtu_maps_voxel_layer_test PRIVATE lingtu_maps)
  add_test(NAME lingtu_maps_voxel_layer_test COMMAND lingtu_maps_voxel_layer_test)

  add_executable(
    lingtu_maps_rolling_occupancy_test
    tests/cpp/rolling_occupancy_test.cpp
  )
  target_link_libraries(lingtu_maps_rolling_occupancy_test PRIVATE lingtu_maps)
  add_test(
    NAME lingtu_maps_rolling_occupancy_test
    COMMAND lingtu_maps_rolling_occupancy_test
  )

  add_executable(
    lingtu_maps_exploration_projection_test
    tests/cpp/exploration_projection_test.cpp
  )
  target_link_libraries(lingtu_maps_exploration_projection_test PRIVATE lingtu_maps)
  add_test(
    NAME lingtu_maps_exploration_projection_test
    COMMAND lingtu_maps_exploration_projection_test
  )
  add_executable(
    lingtu_maps_semantic_occupancy_test
    tests/cpp/semantic_occupancy_test.cpp
  )
  target_link_libraries(lingtu_maps_semantic_occupancy_test PRIVATE lingtu_maps)
  add_test(
    NAME lingtu_maps_semantic_occupancy_test
    COMMAND lingtu_maps_semantic_occupancy_test
  )

  add_executable(
    lingtu_maps_semantic_map_persistence_test
    tests/cpp/semantic_map_persistence_test.cpp
  )
  target_link_libraries(lingtu_maps_semantic_map_persistence_test PRIVATE lingtu_maps)
  add_test(
    NAME lingtu_maps_semantic_map_persistence_test
    COMMAND lingtu_maps_semantic_map_persistence_test
  )

  add_executable(
    lingtu_maps_semantic_performance_test
    tests/cpp/semantic_performance_test.cpp
  )
  target_link_libraries(lingtu_maps_semantic_performance_test PRIVATE lingtu_maps)
  add_test(
    NAME lingtu_maps_semantic_performance_test
    COMMAND lingtu_maps_semantic_performance_test
  )

  add_executable(lingtu_maps_store_test tests/cpp/store_test.cpp)
  target_link_libraries(lingtu_maps_store_test PRIVATE lingtu_maps)
  add_test(NAME lingtu_maps_store_test COMMAND lingtu_maps_store_test)

  add_executable(lingtu_maps_map_health_test tests/cpp/map_health_test.cpp)
  target_link_libraries(lingtu_maps_map_health_test PRIVATE lingtu_maps)
  add_test(NAME lingtu_maps_map_health_test COMMAND lingtu_maps_map_health_test)

  add_executable(lingtu_maps_artifact_jobs_test tests/cpp/artifact_jobs_test.cpp)
  target_link_libraries(lingtu_maps_artifact_jobs_test PRIVATE lingtu_maps)
  add_test(NAME lingtu_maps_artifact_jobs_test COMMAND lingtu_maps_artifact_jobs_test)

  add_executable(
    lingtu_maps_block_grid_scene_test
    tests/cpp/block_grid_scene_test.cpp
  )
  target_link_libraries(lingtu_maps_block_grid_scene_test PRIVATE lingtu_maps)
  add_test(
    NAME lingtu_maps_block_grid_scene_test
    COMMAND lingtu_maps_block_grid_scene_test
  )

  add_executable(
    lingtu_maps_maintenance_test
    tests/cpp/map_maintenance_test.cpp
  )
  target_link_libraries(lingtu_maps_maintenance_test PRIVATE lingtu_maps)
  add_test(
    NAME lingtu_maps_maintenance_test
    COMMAND lingtu_maps_maintenance_test
  )

  add_executable(lingtu_maps_map_graph_test tests/cpp/map_graph_test.cpp)
  target_link_libraries(lingtu_maps_map_graph_test PRIVATE lingtu_maps)
  add_test(NAME lingtu_maps_map_graph_test COMMAND lingtu_maps_map_graph_test)

  add_executable(lingtu_maps_active_slots_test tests/cpp/active_slots_test.cpp)
  target_link_libraries(lingtu_maps_active_slots_test PRIVATE lingtu_maps)
  add_test(NAME lingtu_maps_active_slots_test COMMAND lingtu_maps_active_slots_test)

  add_executable(lingtu_maps_save_map_test tests/cpp/save_map_test.cpp)
  target_link_libraries(lingtu_maps_save_map_test PRIVATE lingtu_maps)
  add_test(NAME lingtu_maps_save_map_test COMMAND lingtu_maps_save_map_test)

  add_executable(lingtu_maps_save_map_c_api_test tests/cpp/save_map_c_api_test.cpp)
  target_link_libraries(
    lingtu_maps_save_map_c_api_test
    PRIVATE
      lingtu_maps_c_api
      lingtu_maps_contract
  )
  add_test(NAME lingtu_maps_save_map_c_api_test COMMAND lingtu_maps_save_map_c_api_test)

  add_executable(lingtu_maps_store_c_api_test tests/cpp/store_c_api_test.cpp)
  target_link_libraries(
    lingtu_maps_store_c_api_test
    PRIVATE
      lingtu_maps_c_api
      lingtu_maps_contract
  )
  add_test(NAME lingtu_maps_store_c_api_test COMMAND lingtu_maps_store_c_api_test)

  add_executable(lingtu_maps_service_c_api_test tests/cpp/service_c_api_test.cpp)
  target_link_libraries(
    lingtu_maps_service_c_api_test
    PRIVATE
      lingtu_maps_c_api
      lingtu_maps_contract
  )
  add_test(NAME lingtu_maps_service_c_api_test COMMAND lingtu_maps_service_c_api_test)

  add_executable(lingtu_maps_voxel_c_api_test tests/cpp/voxel_c_api_test.cpp)
  target_link_libraries(
    lingtu_maps_voxel_c_api_test
    PRIVATE
      lingtu_maps_c_api
      lingtu_maps_contract
  )
  add_test(NAME lingtu_maps_voxel_c_api_test COMMAND lingtu_maps_voxel_c_api_test)

  add_executable(lingtu_maps_pcd_c_api_test tests/cpp/pcd_c_api_test.cpp)
  target_link_libraries(
    lingtu_maps_pcd_c_api_test
    PRIVATE
      lingtu_maps_c_api
      lingtu_maps_contract
      lingtu_maps
  )
  add_test(NAME lingtu_maps_pcd_c_api_test COMMAND lingtu_maps_pcd_c_api_test)

  add_executable(lingtu_maps_semantic_c_api_test tests/cpp/semantic_c_api_test.cpp)
  target_link_libraries(
    lingtu_maps_semantic_c_api_test
    PRIVATE
      lingtu_maps_c_api
      lingtu_maps_contract
  )
  add_test(NAME lingtu_maps_semantic_c_api_test COMMAND lingtu_maps_semantic_c_api_test)

  add_executable(
    lingtu_maps_semantic_localization_client_test
    tests/cpp/semantic_localization_client_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../localization/slam/cpp/semantic_map_client.cpp
  )
  target_include_directories(
    lingtu_maps_semantic_localization_client_test
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/../localization/slam/cpp
  )
  target_link_libraries(
    lingtu_maps_semantic_localization_client_test
    PRIVATE
      lingtu_maps_c_api
      lingtu_maps_contract
      ${CMAKE_DL_LIBS}
  )
  add_test(
    NAME lingtu_maps_semantic_localization_client_test
    COMMAND lingtu_maps_semantic_localization_client_test
  )
  set_tests_properties(
    lingtu_maps_semantic_localization_client_test
    PROPERTIES ENVIRONMENT "LINGTU_MAPS_LIB=$<TARGET_FILE:lingtu_maps_c_api>"
  )

  add_executable(lingtu_maps_grid_c_api_test tests/cpp/grid_c_api_test.cpp)
  target_link_libraries(
    lingtu_maps_grid_c_api_test
    PRIVATE
      lingtu_maps_c_api
      lingtu_maps_contract
  )
  add_test(NAME lingtu_maps_grid_c_api_test COMMAND lingtu_maps_grid_c_api_test)

  set(LINGTU_MAPS_TEST_TARGETS
    lingtu_maps_header_smoke
    lingtu_maps_grid_layers_test
    lingtu_maps_voxel_layer_test
    lingtu_maps_rolling_occupancy_test
    lingtu_maps_exploration_projection_test
    lingtu_maps_semantic_occupancy_test
    lingtu_maps_semantic_map_persistence_test
    lingtu_maps_semantic_performance_test
    lingtu_maps_store_test
    lingtu_maps_map_health_test
    lingtu_maps_artifact_jobs_test
    lingtu_maps_block_grid_scene_test
    lingtu_maps_maintenance_test
    lingtu_maps_map_graph_test
    lingtu_maps_active_slots_test
    lingtu_maps_save_map_test
    lingtu_maps_save_map_c_api_test
    lingtu_maps_store_c_api_test
    lingtu_maps_service_c_api_test
    lingtu_maps_voxel_c_api_test
    lingtu_maps_pcd_c_api_test
    lingtu_maps_semantic_c_api_test
    lingtu_maps_semantic_localization_client_test
    lingtu_maps_grid_c_api_test
  )
  foreach(test_target IN LISTS LINGTU_MAPS_TEST_TARGETS)
    if(MSVC)
      target_compile_options(${test_target} PRIVATE /UNDEBUG)
    else()
      target_compile_options(${test_target} PRIVATE -UNDEBUG)
    endif()
  endforeach()
endif()
