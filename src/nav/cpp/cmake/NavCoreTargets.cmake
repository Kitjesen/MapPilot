include_guard(DIRECTORY)

get_filename_component(LINGTU_NAV_CPP_ROOT
  "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
get_filename_component(LINGTU_NAV_ROOT
  "${LINGTU_NAV_CPP_ROOT}/.." ABSOLUTE)
get_filename_component(LINGTU_SRC_ROOT
  "${LINGTU_NAV_ROOT}/.." ABSOLUTE)

set(LINGTU_NAV_INCLUDE_DIR "${LINGTU_NAV_CPP_ROOT}/include")
set(LINGTU_NAV_LOCAL_CPP_DIR "${LINGTU_NAV_CPP_ROOT}/planning/local")
set(LINGTU_NAV_PLAN_CPP_DIR "${LINGTU_NAV_CPP_ROOT}/engine")
set(LINGTU_NAV_GLOBAL_CPP_DIR "${LINGTU_NAV_CPP_ROOT}/planning/global")
set(LINGTU_NAV_CLIENT_CPP_DIR "${LINGTU_NAV_CPP_ROOT}/client")
set(LINGTU_NAV_TESTS_DIR "${LINGTU_NAV_CPP_ROOT}/tests")

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

if(NOT TARGET lingtu_nav_far)
  add_library(lingtu_nav_far STATIC
    "${LINGTU_NAV_GLOBAL_CPP_DIR}/far/far_planner.cpp")
  add_library(LingTuNav::far ALIAS lingtu_nav_far)
  target_compile_features(lingtu_nav_far PUBLIC cxx_std_17)
  target_include_directories(lingtu_nav_far PUBLIC
    "$<BUILD_INTERFACE:${LINGTU_NAV_GLOBAL_CPP_DIR}>"
    "$<INSTALL_INTERFACE:include/nav/global>")
  set_target_properties(lingtu_nav_far PROPERTIES
    POSITION_INDEPENDENT_CODE ON)
  target_compile_options(lingtu_nav_far PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
  lingtu_nav_enable_performance(lingtu_nav_far)
endif()

if(NOT TARGET lingtu_nav_far_c_api)
  add_library(lingtu_nav_far_c_api SHARED
    "${LINGTU_NAV_GLOBAL_CPP_DIR}/far/far_c_api.cpp")
  add_library(LingTuNav::far_c_api ALIAS lingtu_nav_far_c_api)
  target_compile_features(lingtu_nav_far_c_api PUBLIC cxx_std_17)
  target_include_directories(lingtu_nav_far_c_api PUBLIC
    "$<BUILD_INTERFACE:${LINGTU_NAV_GLOBAL_CPP_DIR}>"
    "$<INSTALL_INTERFACE:include/nav/global>")
  target_link_libraries(lingtu_nav_far_c_api PRIVATE lingtu_nav_far)
  target_compile_definitions(lingtu_nav_far_c_api PRIVATE
    LINGTU_NAV_FAR_C_API_BUILD)
  set_target_properties(lingtu_nav_far_c_api PROPERTIES
    OUTPUT_NAME "nav_far")
  lingtu_nav_enable_performance(lingtu_nav_far_c_api)
endif()

# Kept as a CMake variable during the include-path migration. There is no
# longer a C++ source tree under src/nav/kernel.
set(LINGTU_NAV_KERNEL_DIR "${LINGTU_NAV_CPP_ROOT}")

if(NOT TARGET lingtu_nav_local_planner)
  add_library(lingtu_nav_local_planner INTERFACE)
  add_library(LingTuNav::local_planner ALIAS lingtu_nav_local_planner)
  add_library(local_planner_cpp ALIAS lingtu_nav_local_planner)
  target_compile_features(lingtu_nav_local_planner INTERFACE cxx_std_17)
  target_include_directories(lingtu_nav_local_planner INTERFACE
    "$<BUILD_INTERFACE:${LINGTU_NAV_LOCAL_CPP_DIR}>"
    "$<BUILD_INTERFACE:${LINGTU_NAV_INCLUDE_DIR}>"
    "$<INSTALL_INTERFACE:include>")
  target_compile_options(lingtu_nav_local_planner INTERFACE
    $<$<CXX_COMPILER_ID:MSVC>:/utf-8>)
  if(OpenMP_CXX_FOUND)
    target_link_libraries(lingtu_nav_local_planner INTERFACE
      OpenMP::OpenMP_CXX)
  endif()
endif()

if(NOT TARGET lingtu_nav_path_follower)
  add_library(lingtu_nav_path_follower STATIC
    "${LINGTU_NAV_CPP_ROOT}/control/path_follower_core.cpp")
  add_library(LingTuNav::path_follower ALIAS lingtu_nav_path_follower)
  target_compile_features(lingtu_nav_path_follower PUBLIC cxx_std_17)
  target_include_directories(lingtu_nav_path_follower PUBLIC
    "$<BUILD_INTERFACE:${LINGTU_NAV_INCLUDE_DIR}>"
    "$<INSTALL_INTERFACE:include>")
  set_target_properties(lingtu_nav_path_follower PROPERTIES
    POSITION_INDEPENDENT_CODE ON)
  target_compile_options(lingtu_nav_path_follower PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
  lingtu_nav_enable_performance(lingtu_nav_path_follower)
endif()

if(NOT TARGET lingtu_nav_plan_loop)
  add_library(lingtu_nav_plan_loop STATIC
    "${LINGTU_NAV_PLAN_CPP_DIR}/nav_loop.cpp")
  add_library(LingTuNav::plan_loop ALIAS lingtu_nav_plan_loop)
  target_compile_features(lingtu_nav_plan_loop PUBLIC cxx_std_17)
  target_include_directories(lingtu_nav_plan_loop PUBLIC
    "$<BUILD_INTERFACE:${LINGTU_NAV_PLAN_CPP_DIR}>"
    "$<BUILD_INTERFACE:${LINGTU_NAV_LOCAL_CPP_DIR}>"
    "$<BUILD_INTERFACE:${LINGTU_NAV_INCLUDE_DIR}>")
  target_link_libraries(lingtu_nav_plan_loop PUBLIC
    lingtu_nav_path_follower
    lingtu_nav_local_planner)
  set_target_properties(lingtu_nav_plan_loop PROPERTIES
    POSITION_INDEPENDENT_CODE ON)
  target_compile_options(lingtu_nav_plan_loop PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
  lingtu_nav_enable_performance(lingtu_nav_plan_loop)
endif()
