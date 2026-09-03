# Native endpoint targets are owned by src/nav/cpp/CMakeLists.txt.

find_package(iceoryx_binding_c QUIET)
find_package(CycloneDDS REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenMP QUIET)
include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
option(
  LINGTU_NAV_ENDPOINT_BUILD_OCTOPLANNER
  "Build OctoPlanner-backed endpoint targets"
  ON)

set(_LINGTU_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../../..")
set(_LINGTU_IDL "${_LINGTU_ROOT}/src/message/idl/messages.idl")
set(_NAV_CPP_DIR "${_LINGTU_ROOT}/src/nav/cpp")
set(_NAV_COMMANDS_CPP_DIR "${_NAV_CPP_DIR}/client")
set(_EXPLORE_CPP_DIR "${_LINGTU_ROOT}/src/explore/cpp")
set(_EXPLORE_ENDPOINT_DIR "${_EXPLORE_CPP_DIR}/endpoint")
set(_PERCEPTION_INSPECTION_DIR "${_LINGTU_ROOT}/src/perception/inspection")
set(_OCTOPLANNER3D_RUNTIME_DIR
  "${_NAV_CPP_DIR}/planning/global/octoplanner")

include("${_NAV_CPP_DIR}/cmake/NavCoreTargets.cmake")
set(_NAV_CORE_DIR "${LINGTU_NAV_CPP_ROOT}")
set(_NAV_KERNEL_DIR "${LINGTU_NAV_KERNEL_DIR}")
set(_EP_DIRS
  "${CMAKE_CURRENT_LIST_DIR}"
  "${CMAKE_CURRENT_LIST_DIR}/nav"
  "${_EXPLORE_CPP_DIR}"
  "${_LINGTU_ROOT}/src")

if(NOT TARGET lingtu_maps)
  set(_LINGTU_NAV_SAVED_BUILD_TESTING "${BUILD_TESTING}")
  set(BUILD_TESTING OFF)
  add_subdirectory(
    "${_LINGTU_ROOT}/src/maps"
    "${CMAKE_BINARY_DIR}/maps"
    EXCLUDE_FROM_ALL)
  set(BUILD_TESTING "${_LINGTU_NAV_SAVED_BUILD_TESTING}")
  unset(_LINGTU_NAV_SAVED_BUILD_TESTING)
endif()
if(NOT TARGET lingtu_inspection_core)
  set(_LINGTU_NAV_SAVED_BUILD_TESTING "${BUILD_TESTING}")
  set(BUILD_TESTING "${LINGTU_NAV_CPP_BUILD_TESTS}")
  add_subdirectory(
    "${_LINGTU_ROOT}/src/nav/inspection"
    "${CMAKE_BINARY_DIR}/inspection")
  set(BUILD_TESTING "${_LINGTU_NAV_SAVED_BUILD_TESTING}")
  unset(_LINGTU_NAV_SAVED_BUILD_TESTING)
endif()
if(NOT TARGET lingtu_dds_contracts)
  add_subdirectory(
    "${_LINGTU_ROOT}/src/message/cpp"
    "${CMAKE_BINARY_DIR}/message_cpp")
endif()
lingtu_add_dds_c_messages(lingtu_dds_messages "${_LINGTU_IDL}")

# Windows resolves dependent DLLs from the executable directory before PATH.
# Keep every endpoint-owned executable and shared library in one configuration
# directory so tests, tools, and Product binaries share the same app-local
# runtime closure. External subprojects above retain their own output layout.
if(WIN32)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
endif()

function(lingtu_sources_exist out_var)
  set(_ok TRUE)
  foreach(_path IN LISTS ARGN)
    if(NOT EXISTS "${_path}")
      set(_ok FALSE)
    endif()
  endforeach()
  set("${out_var}" "${_ok}" PARENT_SCOPE)
endfunction()

if(LINGTU_NAV_ENDPOINT_BUILD_OCTOPLANNER
   AND EXISTS "${_OCTOPLANNER3D_RUNTIME_DIR}/CMakeLists.txt")
  set(
    OCTOPLANNER3D_RUNTIME_ENABLE_PCD_INPUT
    OFF
    CACHE BOOL
    "Keep PCL/VTK out of the online navigation endpoint"
    FORCE)
  add_subdirectory(
    "${_OCTOPLANNER3D_RUNTIME_DIR}"
    "${CMAKE_BINARY_DIR}/octoplanner3d_runtime")
  set(_HAS_OCTOPLANNER3D_RUNTIME TRUE)
else()
  set(_HAS_OCTOPLANNER3D_RUNTIME FALSE)
endif()

add_library(lingtu_nav_client SHARED
  "${_NAV_COMMANDS_CPP_DIR}/client.cpp"
  "${_NAV_COMMANDS_CPP_DIR}/client_c.cpp")

target_include_directories(lingtu_nav_client PUBLIC
  "${CMAKE_CURRENT_BINARY_DIR}"
  "${_LINGTU_ROOT}/src"
  "${_NAV_COMMANDS_CPP_DIR}")

target_link_libraries(lingtu_nav_client PRIVATE
  lingtu_dds_messages
  CycloneDDS::ddsc
  lingtu_dds_contracts
  Threads::Threads)

target_compile_definitions(lingtu_nav_client PRIVATE
  LINGTU_NAV_CLIENT_BUILD)

lingtu_endpoint_enable_warnings(lingtu_nav_client)
set_target_properties(lingtu_nav_client PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
  INSTALL_RPATH "$ORIGIN")

add_library(lingtu_inspection_evidence_bridge SHARED
  "${_PERCEPTION_INSPECTION_DIR}/native_bridge.cpp")

target_include_directories(lingtu_inspection_evidence_bridge PUBLIC
  "${CMAKE_CURRENT_BINARY_DIR}"
  "${_LINGTU_ROOT}/src")

target_link_libraries(lingtu_inspection_evidence_bridge PRIVATE
  lingtu_dds_messages
  CycloneDDS::ddsc
  lingtu_dds_contracts
  Threads::Threads)

target_compile_definitions(lingtu_inspection_evidence_bridge PRIVATE
  LINGTU_INSPECTION_EVIDENCE_BRIDGE_BUILD)

lingtu_endpoint_enable_warnings(lingtu_inspection_evidence_bridge)
set_target_properties(lingtu_inspection_evidence_bridge PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
  INSTALL_RPATH "$ORIGIN")

if(_HAS_OCTOPLANNER3D_RUNTIME)
  add_executable(navd
    nav/input/active/octomap.cpp
    nav/input/active/occupancy.cpp
    nav/input/active/declared.cpp
    nav/main.cpp
    nav/runtime/loop.cpp
    nav/input/pose.cpp
    nav/input/map.cpp
    nav/input/health.cpp
    nav/input/planner.cpp
    nav/runtime/inspection/inspection_command_coordinator.cpp
    nav/runtime/inspection/inspection_runtime_controller.cpp
    nav/runtime/goal/blockage.cpp
    nav/runtime/goal/task.cpp
    nav/runtime/goal/retry.cpp
    nav/runtime/goal/plan.cpp
    nav/runtime/goal/runtime.cpp
    nav/command/goal.cpp
    nav/command/terminal.cpp
    nav/dds/runtime.cpp
    nav/runtime/config/parse.cpp
    nav/runtime/config/build.cpp
    nav/input/gate.cpp
    nav/dds/codec.cpp
    nav/dds/frame.cpp
    nav/input/frame.cpp
    nav/control/guard.cpp
    nav/control/authority.cpp
    nav/control/operator.cpp
    nav/status/control_loop_health.cpp
    nav/status/nav_status_writer.cpp
    nav/status/nav_status_publisher.cpp
    nav/status/nav_status_endpoint_adapter.cpp
    nav/status/navigation_goal_status_outbox.cpp
    nav/status/goal_terminal_status_delivery.cpp
    nav/status/goal_terminal_transaction.cpp
    nav/runtime/navigation.cpp
    nav/status/navigation_state.cpp
    nav/status/status_snapshot_file_writer.cpp
    nav/status/inspection_status_file_writer.cpp
    nav/command/cancel.cpp
    nav/command/ingress.cpp
    nav/safety/geofence.cpp
    nav/safety/stop.cpp
    nav/control/admission.cpp
    nav/control/final.cpp
    nav/control/teleop.cpp
    nav/control/autonomy.cpp
    nav/input/obstacle.cpp
    nav/safety/command.cpp
  )

  target_include_directories(navd PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_LINGTU_ROOT}/src"
    "${_NAV_CORE_DIR}"
    "${_NAV_CORE_DIR}"
    "${_NAV_KERNEL_DIR}/include"
    "${_EXPLORE_CPP_DIR}"
    "${_OCTOPLANNER3D_RUNTIME_DIR}"
    ${_EP_DIRS})

  target_link_libraries(navd PRIVATE
    lingtu_inspection_core
    lingtu_maps
    lingtu_nav_far
    lingtu_nav_navigation
    lingtu_nav_rolling_segment
    lingtu_nav_velocity_smoother
    lingtu_dds_messages
    octoplanner3d_runtime
    CycloneDDS::ddsc
    lingtu_dds_contracts
    Threads::Threads)

  if(OpenMP_CXX_FOUND)
    target_link_libraries(navd PRIVATE OpenMP::OpenMP_CXX)
  endif()

  lingtu_endpoint_enable_warnings(navd)
  lingtu_nav_enable_performance(navd)
  set_target_properties(navd PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
elseif(LINGTU_NAV_ENDPOINT_BUILD_OCTOPLANNER)
  message(FATAL_ERROR "Cannot build required navd target: OctoPlanner3D runtime is missing")
endif()

add_executable(lingtu_traversability_dds
    nav/dds/frame.cpp
    nav/input/frame.cpp
    nav/input/gate.cpp
    traversability/main.cpp
    traversability/rolling_exploration_map.cpp
  )

  target_include_directories(lingtu_traversability_dds PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_LINGTU_ROOT}/src"
    "${_LINGTU_ROOT}/src/maps/include"
    "${_NAV_KERNEL_DIR}/include"
    ${_EP_DIRS})

  target_link_libraries(lingtu_traversability_dds PRIVATE
    lingtu_dds_messages
    lingtu_maps
    CycloneDDS::ddsc
    lingtu_dds_contracts
    Threads::Threads)

  if(OpenMP_CXX_FOUND)
    target_link_libraries(lingtu_traversability_dds PRIVATE OpenMP::OpenMP_CXX)
  endif()

  lingtu_endpoint_enable_warnings(lingtu_traversability_dds)
  lingtu_nav_enable_performance(lingtu_traversability_dds)
  set_target_properties(lingtu_traversability_dds PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

add_executable(lingtu_explore_dds
    nav/dds/frame.cpp
    nav/input/frame.cpp
    "${_EXPLORE_ENDPOINT_DIR}/main.cpp"
    "${_EXPLORE_ENDPOINT_DIR}/control.cpp"
    "${_EXPLORE_ENDPOINT_DIR}/run_event_outbox.cpp"
    "${_EXPLORE_ENDPOINT_DIR}/run_lifecycle.cpp"
    "${_EXPLORE_ENDPOINT_DIR}/directed_intent.cpp"
    "${_EXPLORE_ENDPOINT_DIR}/goal_command_lane.cpp"
    "${_EXPLORE_ENDPOINT_DIR}/goal_lifecycle.cpp"
    "${_EXPLORE_ENDPOINT_DIR}/input_gate.cpp"
    "${_EXPLORE_ENDPOINT_DIR}/saved_coverage_grid.cpp"
    nav/input/active/occupancy.cpp
    nav/input/active/declared.cpp
    nav/status/status_snapshot_file_writer.cpp
    "${_EXPLORE_CPP_DIR}/frontier.cpp"
    "${_EXPLORE_CPP_DIR}/keypose_graph.cpp"
    "${_EXPLORE_CPP_DIR}/tare_policy.cpp")

  target_include_directories(lingtu_explore_dds PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_LINGTU_ROOT}/src"
    "${_EXPLORE_CPP_DIR}"
    "${_NAV_CORE_DIR}"
    "${_NAV_KERNEL_DIR}/include"
    ${_EP_DIRS})

  target_link_libraries(lingtu_explore_dds PRIVATE
    lingtu_dds_messages
    lingtu_maps
    lingtu_nav_far
    CycloneDDS::ddsc
    lingtu_dds_contracts
    Threads::Threads)

  lingtu_endpoint_enable_warnings(lingtu_explore_dds)
  set_target_properties(lingtu_explore_dds PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

add_executable(lingtu_nav_control
    tools/navctl.cpp)

  target_include_directories(lingtu_nav_control PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_LINGTU_ROOT}/src"
    ${_EP_DIRS})

  target_link_libraries(lingtu_nav_control PRIVATE
    lingtu_nav_client
    lingtu_dds_messages
    CycloneDDS::ddsc
    lingtu_dds_contracts
    Threads::Threads)

  lingtu_endpoint_enable_warnings(lingtu_nav_control)
  set_target_properties(lingtu_nav_control PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    BUILD_RPATH "$ORIGIN")

add_executable(lingtu_motion_mock_dds
    tools/mock.cpp)

  target_include_directories(lingtu_motion_mock_dds PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_LINGTU_ROOT}/src"
    ${_EP_DIRS})

  target_link_libraries(lingtu_motion_mock_dds PRIVATE
    lingtu_dds_messages
    CycloneDDS::ddsc
    lingtu_dds_contracts
    Threads::Threads)

  lingtu_endpoint_enable_warnings(lingtu_motion_mock_dds)
  set_target_properties(lingtu_motion_mock_dds PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

set(_LINGTU_NAV_ENDPOINT_RUNTIME_TARGETS)
foreach(_target IN ITEMS
    navd
    lingtu_traversability_dds
    lingtu_explore_dds
    lingtu_nav_control)
  if(TARGET ${_target})
    list(APPEND _LINGTU_NAV_ENDPOINT_RUNTIME_TARGETS ${_target})
    set_target_properties(${_target} PROPERTIES
      INSTALL_RPATH "$ORIGIN/../${CMAKE_INSTALL_LIBDIR}")
  endif()
endforeach()
if(_LINGTU_NAV_ENDPOINT_RUNTIME_TARGETS)
  install(TARGETS ${_LINGTU_NAV_ENDPOINT_RUNTIME_TARGETS}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
      COMPONENT lingtu_runtime)
endif()

set(_LINGTU_CMU_PATH_SOURCE "${_NAV_CPP_DIR}/planning/local/cmu/paths")
set(_LINGTU_CMU_PATH_BUILD "${CMAKE_BINARY_DIR}/cmu_paths")
add_custom_target(lingtu_cmu_paths ALL
  COMMAND ${CMAKE_COMMAND} -E make_directory "${_LINGTU_CMU_PATH_BUILD}"
  COMMAND ${CMAKE_COMMAND} -E copy_directory
    "${_LINGTU_CMU_PATH_SOURCE}/go2"
    "${_LINGTU_CMU_PATH_BUILD}/go2"
  COMMAND ${CMAKE_COMMAND} -E copy_directory
    "${_LINGTU_CMU_PATH_SOURCE}/thunder"
    "${_LINGTU_CMU_PATH_BUILD}/thunder"
  COMMENT "Staging CMU path libraries for Go2 and Thunder")

install(DIRECTORY
  "${_LINGTU_CMU_PATH_SOURCE}/go2"
  "${_LINGTU_CMU_PATH_SOURCE}/thunder"
  DESTINATION ${CMAKE_INSTALL_DATADIR}/lingtu/cmu_paths
  COMPONENT cmu_paths)

if(WIN32)
  # Do not maintain a hand-written allowlist: every endpoint-owned executable
  # or shared library that directly links CycloneDDS must be independently
  # runnable after a targeted build. This also covers new DDS tests and tools.
  get_property(
    _LINGTU_NAV_ENDPOINT_DIRECTORY_TARGETS
    DIRECTORY
    PROPERTY BUILDSYSTEM_TARGETS)
  foreach(_target IN LISTS _LINGTU_NAV_ENDPOINT_DIRECTORY_TARGETS)
    get_target_property(_target_type ${_target} TYPE)
    if(_target_type STREQUAL "EXECUTABLE"
        OR _target_type STREQUAL "SHARED_LIBRARY"
        OR _target_type STREQUAL "MODULE_LIBRARY")
      get_target_property(_target_link_libraries ${_target} LINK_LIBRARIES)
      if(_target_link_libraries)
        list(FIND _target_link_libraries "CycloneDDS::ddsc" _ddsc_link_index)
        if(NOT _ddsc_link_index EQUAL -1)
          lingtu_endpoint_bundle_cyclonedds(${_target})
        endif()
      endif()
    endif()
  endforeach()
  unset(_ddsc_link_index)
  unset(_target_link_libraries)
  unset(_target_type)
  unset(_LINGTU_NAV_ENDPOINT_DIRECTORY_TARGETS)
endif()

install(TARGETS
  lingtu_nav_client
  lingtu_inspection_evidence_bridge
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    COMPONENT lingtu_runtime
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    COMPONENT lingtu_runtime)
