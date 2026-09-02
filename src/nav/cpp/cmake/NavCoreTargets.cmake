include_guard(DIRECTORY)

get_filename_component(LINGTU_NAV_CPP_ROOT
  "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
get_filename_component(LINGTU_NAV_ROOT
  "${LINGTU_NAV_CPP_ROOT}/.." ABSOLUTE)
get_filename_component(LINGTU_SRC_ROOT
  "${LINGTU_NAV_ROOT}/.." ABSOLUTE)

set(LINGTU_NAV_INCLUDE_DIR "${LINGTU_NAV_CPP_ROOT}/include")
set(LINGTU_NAV_LOCAL_PLANNING_DIR "${LINGTU_NAV_CPP_ROOT}/planning/local")
set(LINGTU_NAV_GLOBAL_PLANNING_DIR "${LINGTU_NAV_CPP_ROOT}/planning/global")
set(LINGTU_NAV_TRACKING_DIR "${LINGTU_NAV_CPP_ROOT}/tracking")
set(LINGTU_NAV_TRAJECTORY_DIR "${LINGTU_NAV_CPP_ROOT}/trajectory")
set(LINGTU_NAV_NAVIGATION_DIR "${LINGTU_NAV_CPP_ROOT}/navigation")
set(LINGTU_NAV_CLIENT_CPP_DIR "${LINGTU_NAV_CPP_ROOT}/client")

include(CheckIPOSupported)
check_ipo_supported(
  RESULT LINGTU_NAV_IPO_SUPPORTED
  OUTPUT LINGTU_NAV_IPO_ERROR
  LANGUAGES CXX)

function(lingtu_nav_enable_performance target)
  target_compile_options(${target} PRIVATE
    $<$<AND:$<CONFIG:Release>,$<COMPILE_LANG_AND_ID:CXX,GNU,Clang,AppleClang>>:-funroll-loops>)
  if(LINGTU_NAV_IPO_SUPPORTED)
    set_target_properties(${target} PROPERTIES
      INTERPROCEDURAL_OPTIMIZATION_RELEASE ON
      INTERPROCEDURAL_OPTIMIZATION_RELWITHDEBINFO ON)
  endif()
endfunction()

find_package(OpenMP QUIET)

if(WIN32)
  set(_LINGTU_EXISTING_NATIVE_DEPS
    "${LINGTU_SRC_ROOT}/../third_party/install/slam-windows/x64-windows")
  if(EXISTS "${_LINGTU_EXISTING_NATIVE_DEPS}/share/eigen3/Eigen3Config.cmake")
    list(PREPEND CMAKE_PREFIX_PATH "${_LINGTU_EXISTING_NATIVE_DEPS}")
  endif()
endif()
find_package(Eigen3 REQUIRED CONFIG)

if(NOT TARGET lingtu_nav_far)
  add_library(lingtu_nav_far STATIC
    "${LINGTU_NAV_GLOBAL_PLANNING_DIR}/far/planner.cpp")
  add_library(LingTuNav::far ALIAS lingtu_nav_far)
  target_compile_features(lingtu_nav_far PUBLIC cxx_std_17)
  target_include_directories(lingtu_nav_far PUBLIC
    "$<BUILD_INTERFACE:${LINGTU_NAV_CPP_ROOT}>")
  set_target_properties(lingtu_nav_far PROPERTIES
    POSITION_INDEPENDENT_CODE ON)
  target_compile_options(lingtu_nav_far PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
  lingtu_nav_enable_performance(lingtu_nav_far)
endif()

if(NOT TARGET lingtu_nav_far_c_api)
  add_library(lingtu_nav_far_c_api SHARED
    "${LINGTU_NAV_GLOBAL_PLANNING_DIR}/far/api.cpp")
  add_library(LingTuNav::far_c_api ALIAS lingtu_nav_far_c_api)
  target_compile_features(lingtu_nav_far_c_api PUBLIC cxx_std_17)
  target_include_directories(lingtu_nav_far_c_api PUBLIC
    "$<BUILD_INTERFACE:${LINGTU_NAV_CPP_ROOT}>")
  target_link_libraries(lingtu_nav_far_c_api PRIVATE lingtu_nav_far)
  target_compile_definitions(lingtu_nav_far_c_api PRIVATE
    LINGTU_NAV_FAR_C_API_BUILD)
  set_target_properties(lingtu_nav_far_c_api PROPERTIES
    OUTPUT_NAME "nav_far")
  lingtu_nav_enable_performance(lingtu_nav_far_c_api)
endif()

# Shared navigation source root consumed by endpoint and test targets.
set(LINGTU_NAV_KERNEL_DIR "${LINGTU_NAV_CPP_ROOT}")

if(NOT TARGET lingtu_nav_spline)
  add_library(lingtu_nav_spline STATIC
    "${LINGTU_NAV_TRAJECTORY_DIR}/spline.cpp"
    "${LINGTU_NAV_LOCAL_PLANNING_DIR}/scan/upstream/bspline_opt/uniform_bspline.cpp"
    "${LINGTU_NAV_LOCAL_PLANNING_DIR}/scan/upstream/plan_manage/closed_loop_controller.cpp")
  target_compile_features(lingtu_nav_spline PUBLIC cxx_std_17)
  target_include_directories(lingtu_nav_spline PUBLIC
    "$<BUILD_INTERFACE:${LINGTU_NAV_CPP_ROOT}>"
    "$<BUILD_INTERFACE:${LINGTU_NAV_INCLUDE_DIR}>")
  target_link_libraries(lingtu_nav_spline PUBLIC Eigen3::Eigen)
  set_target_properties(lingtu_nav_spline PROPERTIES
    POSITION_INDEPENDENT_CODE ON)
  target_compile_options(lingtu_nav_spline PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<CXX_COMPILER_ID:MSVC>:/utf-8>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
  lingtu_nav_enable_performance(lingtu_nav_spline)
endif()

if(NOT TARGET lingtu_nav_local_planner)
  add_library(lingtu_nav_local_planner STATIC
    "${LINGTU_NAV_LOCAL_PLANNING_DIR}/planner.cpp"
    "${LINGTU_NAV_LOCAL_PLANNING_DIR}/task.cpp"
    "${LINGTU_NAV_LOCAL_PLANNING_DIR}/cmu/backend.cpp"
    "${LINGTU_NAV_LOCAL_PLANNING_DIR}/scan/backend.cpp"
    "${LINGTU_NAV_LOCAL_PLANNING_DIR}/scan/grid.cpp"
    "${LINGTU_NAV_LOCAL_PLANNING_DIR}/scan/upstream/bspline_opt/bspline_optimizer.cpp"
    "${LINGTU_NAV_LOCAL_PLANNING_DIR}/scan/upstream/path_searching/dyn_a_star.cpp"
    "${LINGTU_NAV_LOCAL_PLANNING_DIR}/scan/upstream/plan_env/grid_map.cpp"
    "${LINGTU_NAV_LOCAL_PLANNING_DIR}/scan/upstream/plan_manage/planner_manager.cpp"
    "${LINGTU_NAV_LOCAL_PLANNING_DIR}/scan/upstream/plan_manage/scan_replan_fsm.cpp"
    "${LINGTU_NAV_LOCAL_PLANNING_DIR}/scan/upstream/traj_utils/polynomial_traj.cpp"
    "${LINGTU_NAV_LOCAL_PLANNING_DIR}/recovery.cpp")
  add_library(LingTuNav::local_planner ALIAS lingtu_nav_local_planner)
  add_library(local_planner_cpp ALIAS lingtu_nav_local_planner)
  target_compile_features(lingtu_nav_local_planner PUBLIC cxx_std_17)
  target_include_directories(lingtu_nav_local_planner PUBLIC
    "$<BUILD_INTERFACE:${LINGTU_NAV_CPP_ROOT}>"
    "$<BUILD_INTERFACE:${LINGTU_NAV_INCLUDE_DIR}>")
  find_package(Threads REQUIRED)
  target_link_libraries(lingtu_nav_local_planner PUBLIC
    Threads::Threads
    Eigen3::Eigen
    lingtu_nav_spline)
  set_target_properties(lingtu_nav_local_planner PROPERTIES
    POSITION_INDEPENDENT_CODE ON)
  target_compile_options(lingtu_nav_local_planner PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<CXX_COMPILER_ID:MSVC>:/utf-8>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
  if(OpenMP_CXX_FOUND)
    target_link_libraries(lingtu_nav_local_planner PUBLIC
      OpenMP::OpenMP_CXX)
  endif()
  lingtu_nav_enable_performance(lingtu_nav_local_planner)
endif()

if(NOT TARGET lingtu_nav_follower)
  add_library(lingtu_nav_follower STATIC
    "${LINGTU_NAV_TRACKING_DIR}/follower.cpp")
  add_library(LingTuNav::follower ALIAS lingtu_nav_follower)
  target_compile_features(lingtu_nav_follower PUBLIC cxx_std_17)
  target_include_directories(lingtu_nav_follower PUBLIC
    "$<BUILD_INTERFACE:${LINGTU_NAV_CPP_ROOT}>"
    "$<BUILD_INTERFACE:${LINGTU_NAV_INCLUDE_DIR}>")
  target_link_libraries(lingtu_nav_follower PRIVATE lingtu_nav_spline)
  set_target_properties(lingtu_nav_follower PROPERTIES
    POSITION_INDEPENDENT_CODE ON)
  target_compile_options(lingtu_nav_follower PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
  lingtu_nav_enable_performance(lingtu_nav_follower)
endif()

if(NOT TARGET lingtu_nav_navigation)
  add_library(lingtu_nav_navigation STATIC
    "${LINGTU_NAV_NAVIGATION_DIR}/executor.cpp"
    "${LINGTU_NAV_NAVIGATION_DIR}/recovery.cpp"
    "${LINGTU_NAV_NAVIGATION_DIR}/route.cpp"
    "${LINGTU_NAV_NAVIGATION_DIR}/state.cpp")
  add_library(LingTuNav::navigation ALIAS lingtu_nav_navigation)
  target_compile_features(lingtu_nav_navigation PUBLIC cxx_std_17)
  target_include_directories(lingtu_nav_navigation PUBLIC
    "$<BUILD_INTERFACE:${LINGTU_NAV_CPP_ROOT}>"
    "$<BUILD_INTERFACE:${LINGTU_NAV_INCLUDE_DIR}>")
  target_link_libraries(lingtu_nav_navigation PUBLIC
    lingtu_nav_follower
    lingtu_nav_local_planner)
  set_target_properties(lingtu_nav_navigation PROPERTIES
    POSITION_INDEPENDENT_CODE ON)
  target_compile_options(lingtu_nav_navigation PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
  lingtu_nav_enable_performance(lingtu_nav_navigation)
endif()
