cmake_minimum_required(VERSION 3.16)

project(lingtu_maps LANGUAGES CXX)

include(GNUInstallDirs)

find_package(Threads REQUIRED)

option(LINGTU_MAPS_ENABLE_NATIVE_OCTOMAP "Enable embedded OctoMap artifact builder when OctoMap is available" ON)

if(LINGTU_MAPS_ENABLE_NATIVE_OCTOMAP)
  find_package(octomap QUIET)
  if(NOT octomap_FOUND)
    find_path(OCTOMAP_INCLUDE_DIRS NAMES octomap/OcTree.h)
    find_library(OCTOMAP_LIBRARY NAMES octomap)
    find_library(OCTOMATH_LIBRARY NAMES octomath)
    if(OCTOMAP_INCLUDE_DIRS AND OCTOMAP_LIBRARY AND OCTOMATH_LIBRARY)
      set(octomap_FOUND TRUE)
      set(OCTOMAP_LIBRARIES "${OCTOMAP_LIBRARY};${OCTOMATH_LIBRARY}")
    endif()
  endif()
endif()

add_library(lingtu_maps_contract INTERFACE)
target_include_directories(
  lingtu_maps_contract
  INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_compile_features(lingtu_maps_contract INTERFACE cxx_std_17)

add_library(lingtu_maps STATIC
  cpp/block_grid.cpp
  cpp/health.cpp
  cpp/lock.cpp
  cpp/maintenance.cpp
  cpp/map_graph.cpp
  cpp/semantic_taxonomy.cpp
  cpp/build/grid_artifacts.cpp
  cpp/build/occupancy_snapshot.cpp
  cpp/build/pcd.cpp
  cpp/build/pipeline.cpp
  cpp/build/process.cpp
  cpp/semantic_map_persistence.cpp
  cpp/save.cpp
  cpp/service.cpp
  cpp/service_jobs.cpp
  cpp/service_lifecycle.cpp
  cpp/service_places.cpp
  cpp/store.cpp
  cpp/layers/rolling_occupancy.cpp
  cpp/layers/exploration_projection.cpp
  cpp/layers/semantic_occupancy.cpp
  cpp/layers/voxel.cpp
  cpp/mapd/activation.cpp
  cpp/mapd/engine.cpp
  cpp/mapd/query_core.cpp
  cpp/mapd/query_server.cpp
  cpp/mapd/save_coordinator.cpp
  cpp/mapd/service_dispatch.cpp
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
if(MSVC)
  target_compile_options(lingtu_maps PRIVATE /FS)
endif()

option(LINGTU_MAPS_BUILD_MAPD "Build the native typed-DDS mapd process" OFF)

if(LINGTU_MAPS_BUILD_MAPD)
  enable_language(C)
  find_package(CycloneDDS REQUIRED)
  function(lingtu_maps_bundle_cyclonedds target)
    if(WIN32)
      add_custom_command(
        TARGET ${target}
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
          $<TARGET_FILE:CycloneDDS::ddsc>
          $<TARGET_FILE_DIR:${target}>
        COMMENT "Bundling the CycloneDDS runtime DLL for ${target}"
        VERBATIM
      )
    endif()
  endfunction()
  if(NOT TARGET lingtu_dds_contracts)
    add_subdirectory(
      "${CMAKE_CURRENT_SOURCE_DIR}/../message/cpp"
      "${CMAKE_CURRENT_BINARY_DIR}/message_cpp"
    )
  endif()
  set(_LINGTU_MAPD_IDL
    "${CMAKE_CURRENT_SOURCE_DIR}/../message/idl/messages.idl"
  )
  lingtu_add_dds_c_messages(lingtu_mapd_dds_messages "${_LINGTU_MAPD_IDL}")

  add_library(lingtu_maps_mapd_dds STATIC cpp/mapd/dds.cpp)
  target_include_directories(
    lingtu_maps_mapd_dds
    PUBLIC
      "${CMAKE_CURRENT_SOURCE_DIR}/cpp/mapd"
    PRIVATE
      "${CMAKE_CURRENT_SOURCE_DIR}/.."
  )
  target_link_libraries(
    lingtu_maps_mapd_dds
    PUBLIC
      lingtu_maps
      lingtu_mapd_dds_messages
      lingtu_dds_contracts
      Threads::Threads
  )
  set_target_properties(
    lingtu_maps_mapd_dds
    PROPERTIES POSITION_INDEPENDENT_CODE ON
  )

  add_executable(mapd cpp/mapd/main.cpp)
  target_include_directories(
    mapd
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/.."
  )
  target_link_libraries(mapd PRIVATE lingtu_maps_mapd_dds)
  lingtu_maps_bundle_cyclonedds(mapd)
  add_executable(lingtu_mapctl cpp/tools/mapctl.cpp)
  target_include_directories(
    lingtu_mapctl
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/.."
  )
  target_link_libraries(
    lingtu_mapctl
    PRIVATE
      lingtu_maps
      lingtu_mapd_dds_messages
      lingtu_dds_contracts
      Threads::Threads
  )
  set_target_properties(lingtu_mapctl PROPERTIES OUTPUT_NAME "lingtu-mapctl")
  lingtu_maps_bundle_cyclonedds(lingtu_mapctl)
  if(MSVC)
    target_compile_options(lingtu_maps_mapd_dds PRIVATE /W4)
    target_compile_options(mapd PRIVATE /W4)
    target_compile_options(lingtu_mapctl PRIVATE /W4)
  else()
    target_compile_options(
      lingtu_maps_mapd_dds
      PRIVATE -Wall -Wextra -Wpedantic
    )
    target_compile_options(mapd PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(lingtu_mapctl PRIVATE -Wall -Wextra -Wpedantic)
  endif()
  install(TARGETS mapd lingtu_mapctl
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
      COMPONENT lingtu_runtime)
endif()

option(LINGTU_MAPS_BUILD_TOOLS "Build LingTu maps offline command-line tools" OFF)

if(LINGTU_MAPS_BUILD_TOOLS)
  add_executable(
    lingtu_maps_import_unity
    cpp/tools/import_unity_semantics.cpp
    cpp/sources/unity_scene.cpp
    cpp/semantic_taxonomy.cpp
    cpp/semantic_map_persistence.cpp
  )
  target_link_libraries(
    lingtu_maps_import_unity
    PRIVATE
      lingtu_maps_contract
      Threads::Threads
  )
  set_target_properties(lingtu_maps_import_unity PROPERTIES
    OUTPUT_NAME "lingtu-maps-import-unity"
  )
endif()

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

  add_executable(
    lingtu_maps_unity_semantic_import_test
    tests/cpp/unity_semantic_import_test.cpp
    cpp/sources/unity_scene.cpp
    cpp/semantic_taxonomy.cpp
    cpp/semantic_map_persistence.cpp
  )
  target_link_libraries(
    lingtu_maps_unity_semantic_import_test
    PRIVATE
      lingtu_maps_contract
      Threads::Threads
  )
  add_test(
    NAME lingtu_maps_unity_semantic_import_test
    COMMAND lingtu_maps_unity_semantic_import_test
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

  add_executable(lingtu_maps_process_test tests/cpp/process_test.cpp)
  target_link_libraries(lingtu_maps_process_test PRIVATE lingtu_maps)
  add_test(NAME lingtu_maps_process_test COMMAND lingtu_maps_process_test)

  add_executable(lingtu_maps_block_grid_test tests/cpp/block_grid_test.cpp)
  target_link_libraries(lingtu_maps_block_grid_test PRIVATE lingtu_maps)
  add_test(NAME lingtu_maps_block_grid_test COMMAND lingtu_maps_block_grid_test)

  add_executable(
    lingtu_maps_mapd_engine_test
    tests/cpp/mapd_engine_test.cpp
  )
  target_link_libraries(lingtu_maps_mapd_engine_test PRIVATE lingtu_maps)
  add_test(
    NAME lingtu_maps_mapd_engine_test
    COMMAND lingtu_maps_mapd_engine_test
  )

  add_executable(
    lingtu_maps_map_activation_test
    tests/cpp/map_activation_test.cpp
  )

  add_executable(
    lingtu_maps_mapd_service_dispatch_test
    tests/cpp/mapd_service_dispatch_test.cpp
  )
  target_link_libraries(lingtu_maps_mapd_service_dispatch_test PRIVATE lingtu_maps)
  add_test(
    NAME lingtu_maps_mapd_service_dispatch_test
    COMMAND lingtu_maps_mapd_service_dispatch_test
  )
  add_executable(
    lingtu_maps_mapd_save_coordinator_test
    tests/cpp/mapd_save_coordinator_test.cpp
  )
  target_link_libraries(lingtu_maps_mapd_save_coordinator_test PRIVATE lingtu_maps)
  add_test(
    NAME lingtu_maps_mapd_save_coordinator_test
    COMMAND lingtu_maps_mapd_save_coordinator_test
  )
  target_link_libraries(lingtu_maps_map_activation_test PRIVATE lingtu_maps)
  if(LINGTU_MAPS_BUILD_MAPD)
    add_dependencies(lingtu_maps_map_activation_test lingtu_mapctl)
    add_test(
      NAME lingtu_maps_map_activation_test
      COMMAND lingtu_maps_map_activation_test $<TARGET_FILE:lingtu_mapctl>
    )
  else()
    add_test(
      NAME lingtu_maps_map_activation_test
      COMMAND lingtu_maps_map_activation_test
    )
  endif()

  if(LINGTU_MAPS_BUILD_MAPD)
    add_executable(
      lingtu_maps_mapd_dds_test
      tests/cpp/mapd_dds_test.cpp
    )
    target_include_directories(
      lingtu_maps_mapd_dds_test
      PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/cpp/mapd"
        "${CMAKE_CURRENT_SOURCE_DIR}/.."
    )
    target_link_libraries(
      lingtu_maps_mapd_dds_test
      PRIVATE
        lingtu_maps_mapd_dds
    )
    lingtu_maps_bundle_cyclonedds(lingtu_maps_mapd_dds_test)
    add_test(
      NAME lingtu_maps_mapd_dds_test
      COMMAND lingtu_maps_mapd_dds_test
    )
  endif()

  if(UNIX AND NOT APPLE)
    add_executable(
      lingtu_maps_mapd_uds_test
      tests/cpp/mapd_uds_test.cpp
    )
    target_link_libraries(lingtu_maps_mapd_uds_test PRIVATE lingtu_maps)
    add_test(
      NAME lingtu_maps_mapd_uds_test
      COMMAND lingtu_maps_mapd_uds_test
    )
  endif()

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

  add_executable(lingtu_maps_save_map_test tests/cpp/save_map_test.cpp)
  target_link_libraries(lingtu_maps_save_map_test PRIVATE lingtu_maps)
  if(octomap_FOUND)
    target_compile_definitions(lingtu_maps_save_map_test PRIVATE LINGTU_MAPS_HAS_OCTOMAP=1)
  endif()
  add_test(NAME lingtu_maps_save_map_test COMMAND lingtu_maps_save_map_test)

  add_executable(
    lingtu_maps_grid_artifact_io_test
    tests/cpp/grid_artifact_io_test.cpp
  )
  target_link_libraries(lingtu_maps_grid_artifact_io_test PRIVATE lingtu_maps)
  add_test(
    NAME lingtu_maps_grid_artifact_io_test
    COMMAND lingtu_maps_grid_artifact_io_test
  )

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
      lingtu_maps
  )
  add_test(
    NAME lingtu_maps_semantic_localization_client_test
    COMMAND lingtu_maps_semantic_localization_client_test
  )
  set(LINGTU_MAPS_TEST_TARGETS
    lingtu_maps_header_smoke
    lingtu_maps_grid_layers_test
    lingtu_maps_voxel_layer_test
    lingtu_maps_rolling_occupancy_test
    lingtu_maps_semantic_occupancy_test
    lingtu_maps_semantic_map_persistence_test
    lingtu_maps_unity_semantic_import_test
    lingtu_maps_semantic_performance_test
    lingtu_maps_store_test
    lingtu_maps_map_health_test
    lingtu_maps_process_test
    lingtu_maps_block_grid_test
    lingtu_maps_mapd_engine_test
    lingtu_maps_mapd_service_dispatch_test
    lingtu_maps_mapd_save_coordinator_test
    lingtu_maps_map_activation_test
    lingtu_maps_maintenance_test
    lingtu_maps_map_graph_test
    lingtu_maps_save_map_test
    lingtu_maps_grid_artifact_io_test
    lingtu_maps_semantic_localization_client_test
  )
  if(LINGTU_MAPS_BUILD_MAPD)
    list(APPEND LINGTU_MAPS_TEST_TARGETS lingtu_maps_mapd_dds_test)
  endif()
  if(UNIX AND NOT APPLE)
    list(APPEND LINGTU_MAPS_TEST_TARGETS lingtu_maps_mapd_uds_test)
  endif()
  foreach(test_target IN LISTS LINGTU_MAPS_TEST_TARGETS)
    if(MSVC)
      target_compile_options(${test_target} PRIVATE /UNDEBUG /FS)
    else()
      target_compile_options(${test_target} PRIVATE -UNDEBUG)
    endif()
  endforeach()
endif()
