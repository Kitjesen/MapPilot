cmake_minimum_required(VERSION 3.16)
project(lingtu_slam_contract LANGUAGES C CXX)

option(LINGTU_SLAM_BUILD_TESTS "Build native SLAM contract tests" ON)
option(LINGTU_SLAM_FASTLIO2_BACKEND "Build the real Fast-LIO2 algorithm backend" OFF)
option(LINGTU_SLAM_BUILD_PYTHON_BINDINGS "Build native SLAM Python bindings" OFF)
option(LINGTU_SLAM_BUILD_DDS_RUNTIME "Build ROS-free CycloneDDS C++ sensor-to-SLAM runtime" OFF)
option(LINGTU_SLAM_BUILD_CPP_DDS_RUNTIME "Deprecated alias for LINGTU_SLAM_BUILD_DDS_RUNTIME" OFF)
option(LINGTU_SLAM_BUILD_CYCLONE_DDS_RUNTIME "Deprecated alias for LINGTU_SLAM_BUILD_DDS_RUNTIME" OFF)
option(LINGTU_REQUIRE_BBS3D "Fail configure if cpu_bbs3d is not available" OFF)
option(LINGTU_ENABLE_SMALL_GICP "Use small_gicp for native relocalization refine when available" ON)
set(CPU_BBS3D_ROOT "" CACHE PATH "Optional local install prefix for cpu_bbs3d")
set(SMALL_GICP_ROOT "" CACHE PATH "Optional small_gicp source or install prefix")
if(NOT CPU_BBS3D_ROOT AND DEFINED ENV{CPU_BBS3D_ROOT})
  set(CPU_BBS3D_ROOT "$ENV{CPU_BBS3D_ROOT}")
endif()
if(NOT SMALL_GICP_ROOT AND DEFINED ENV{SMALL_GICP_ROOT})
  set(SMALL_GICP_ROOT "$ENV{SMALL_GICP_ROOT}")
endif()
if(NOT CPU_BBS3D_ROOT)
  get_filename_component(_LINGTU_REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../.." ABSOLUTE)
  set(_LINGTU_DEFAULT_BBS3D_ROOT "${_LINGTU_REPO_ROOT}/third_party/3d_bbs/install")
  if(EXISTS "${_LINGTU_DEFAULT_BBS3D_ROOT}")
    set(CPU_BBS3D_ROOT "${_LINGTU_DEFAULT_BBS3D_ROOT}" CACHE PATH
      "Optional local install prefix for cpu_bbs3d" FORCE)
  endif()
endif()
if(NOT SMALL_GICP_ROOT)
  get_filename_component(_LINGTU_REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../.." ABSOLUTE)
  set(_LINGTU_DEFAULT_SMALL_GICP_ROOT
    "${_LINGTU_REPO_ROOT}/third_party/research_localization/small_gicp")
  if(EXISTS "${_LINGTU_DEFAULT_SMALL_GICP_ROOT}/include/small_gicp")
    set(SMALL_GICP_ROOT "${_LINGTU_DEFAULT_SMALL_GICP_ROOT}" CACHE PATH
      "Optional small_gicp source or install prefix" FORCE)
  endif()
endif()

if(DEFINED LINGTU_SLAM_WITH_FASTLIO2)
  set(LINGTU_SLAM_FASTLIO2_BACKEND "${LINGTU_SLAM_WITH_FASTLIO2}" CACHE BOOL
    "Build the real Fast-LIO2 algorithm backend" FORCE)
endif()
if(LINGTU_SLAM_BUILD_CPP_DDS_RUNTIME OR LINGTU_SLAM_BUILD_CYCLONE_DDS_RUNTIME)
  set(LINGTU_SLAM_BUILD_DDS_RUNTIME ON CACHE BOOL
    "Build ROS-free CycloneDDS C++ sensor-to-SLAM runtime" FORCE)
endif()

add_library(lingtu_slam_contract STATIC
  slam.cpp
  fastlio.cpp
  pointlio.cpp
)
target_compile_features(lingtu_slam_contract PUBLIC cxx_std_17)
target_include_directories(lingtu_slam_contract PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
set_target_properties(lingtu_slam_contract PROPERTIES POSITION_INDEPENDENT_CODE ON)

if(LINGTU_SLAM_FASTLIO2_BACKEND)
  set(FASTLIO2_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../fastlio2")
  set(LOCALIZER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../localizer")
  find_package(Eigen3 REQUIRED)
  find_package(PCL REQUIRED COMPONENTS common io filters kdtree registration)
  find_package(yaml-cpp REQUIRED)
  find_package(Threads REQUIRED)
  find_package(OpenMP QUIET)
  find_library(CPU_BBS3D_LIBRARY
    NAMES cpu_bbs3d libcpu_bbs3d.so
    HINTS ${CPU_BBS3D_ROOT}
    PATH_SUFFIXES lib lib64)
  find_path(CPU_BBS3D_INCLUDE_DIR
    NAMES cpu_bbs3d/bbs3d.hpp
    HINTS ${CPU_BBS3D_ROOT}
    PATH_SUFFIXES include)
  if(NOT CPU_BBS3D_LIBRARY OR NOT CPU_BBS3D_INCLUDE_DIR)
    if(LINGTU_REQUIRE_BBS3D)
      message(FATAL_ERROR
        "LINGTU_REQUIRE_BBS3D=ON but cpu_bbs3d was not found. "
        "Set CPU_BBS3D_ROOT to its install prefix.")
    endif()
    message(WARNING
      "BBS3D not found; native SLAM will use seeded ICP relocalization only. "
      "Build it with scripts/build/build_3d_bbs.sh or set CPU_BBS3D_ROOT.")
  else()
    message(STATUS "BBS3D enabled: ${CPU_BBS3D_LIBRARY}")
  endif()
  find_path(SMALL_GICP_INCLUDE_DIR
    NAMES small_gicp/pcl/pcl_registration.hpp
    HINTS ${SMALL_GICP_ROOT}
    PATH_SUFFIXES include
    PATHS /usr/local)
  if(LINGTU_ENABLE_SMALL_GICP AND SMALL_GICP_INCLUDE_DIR AND OpenMP_CXX_FOUND)
    message(STATUS "small_gicp relocalization refine enabled: ${SMALL_GICP_INCLUDE_DIR}")
  else()
    message(STATUS "small_gicp not found, disabled, or OpenMP unavailable; using PCL GICP refine")
    set(SMALL_GICP_INCLUDE_DIR "")
  endif()

  target_sources(lingtu_slam_contract PRIVATE
    map_icp.cpp
    semantic_map_client.cpp
    native_relocalizer.cpp
    "${FASTLIO2_DIR}/src/map_builder/commons.cpp"
    "${FASTLIO2_DIR}/src/map_builder/ieskf.cpp"
    "${FASTLIO2_DIR}/src/map_builder/imu_processor.cpp"
    "${FASTLIO2_DIR}/src/map_builder/ikd_Tree.cpp"
    "${FASTLIO2_DIR}/src/map_builder/lidar_processor.cpp"
    "${FASTLIO2_DIR}/src/map_builder/map_builder.cpp"
    "${LOCALIZER_DIR}/src/localizers/commons.cpp"
    "${LOCALIZER_DIR}/src/localizers/icp_localizer.cpp"
    "${LOCALIZER_DIR}/src/localizers/bbs3d_global_localizer.cpp"
  )
  target_compile_definitions(lingtu_slam_contract PRIVATE
    LINGTU_HAS_FASTLIO2_BACKEND=1)
  target_include_directories(lingtu_slam_contract PRIVATE
    "${FASTLIO2_DIR}/src"
    "${FASTLIO2_DIR}/src/map_builder"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../maps/include")
  target_include_directories(lingtu_slam_contract PUBLIC
    "${LOCALIZER_DIR}/src/localizers"
    ${PCL_INCLUDE_DIRS})
  target_link_libraries(lingtu_slam_contract PUBLIC
    Eigen3::Eigen
    Threads::Threads
    yaml-cpp
    ${PCL_LIBRARIES})
  if(CMAKE_DL_LIBS)
    target_link_libraries(lingtu_slam_contract PUBLIC ${CMAKE_DL_LIBS})
  endif()
  if(CPU_BBS3D_LIBRARY AND CPU_BBS3D_INCLUDE_DIR)
    target_compile_definitions(lingtu_slam_contract PRIVATE LINGTU_ENABLE_BBS3D=1)
    target_include_directories(lingtu_slam_contract PRIVATE ${CPU_BBS3D_INCLUDE_DIR})
    target_link_libraries(lingtu_slam_contract PUBLIC ${CPU_BBS3D_LIBRARY})
  endif()
  if(SMALL_GICP_INCLUDE_DIR)
    # icp_localizer.h selects the concrete backend in the class layout.
    # Consumers must compile that header with the same definition and include
    # path as the library or the C++ ABI is inconsistent.
    target_compile_definitions(lingtu_slam_contract PUBLIC LINGTU_ENABLE_SMALL_GICP=1)
    target_include_directories(lingtu_slam_contract PUBLIC ${SMALL_GICP_INCLUDE_DIR})
  endif()
  if(OpenMP_CXX_FOUND)
    target_compile_definitions(lingtu_slam_contract PRIVATE MP_EN MP_PROC_NUM=4)
    target_link_libraries(lingtu_slam_contract PUBLIC OpenMP::OpenMP_CXX)
  endif()
endif()

if(MSVC)
  target_compile_options(lingtu_slam_contract PRIVATE /W4)
else()
  target_compile_options(lingtu_slam_contract PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(LINGTU_SLAM_BUILD_TESTS)
  enable_testing()
  add_executable(test_slam_contract test_slam.cpp)
  target_link_libraries(test_slam_contract PRIVATE lingtu_slam_contract)
  add_executable(test_relocalization_gate test_relocalization_gate.cpp)
  target_link_libraries(test_relocalization_gate PRIVATE lingtu_slam_contract)
  add_test(
    NAME lingtu_slam_relocalization_gate
    COMMAND test_relocalization_gate)
  add_executable(test_map_tracking_health test_map_tracking_health.cpp)
  target_link_libraries(test_map_tracking_health PRIVATE lingtu_slam_contract)
  if(LINGTU_SLAM_FASTLIO2_BACKEND)
    add_executable(test_ieskf_initialization test_ieskf_initialization.cpp)
    target_include_directories(test_ieskf_initialization PRIVATE
      "${FASTLIO2_DIR}/src"
      "${FASTLIO2_DIR}/src/map_builder")
    target_link_libraries(test_ieskf_initialization PRIVATE lingtu_slam_contract)
    add_test(NAME lingtu_slam_ieskf_initialization COMMAND test_ieskf_initialization)
    add_executable(test_ieskf_update_diagnostics test_ieskf_update_diagnostics.cpp)
    target_include_directories(test_ieskf_update_diagnostics PRIVATE
      "${FASTLIO2_DIR}/src"
      "${FASTLIO2_DIR}/src/map_builder")
    target_link_libraries(test_ieskf_update_diagnostics PRIVATE lingtu_slam_contract)
    add_test(
      NAME lingtu_slam_ieskf_update_diagnostics
      COMMAND test_ieskf_update_diagnostics)
    add_executable(test_fastlio2_mock_flow test_fastlio2_mock_flow.cpp)
    target_link_libraries(test_fastlio2_mock_flow PRIVATE lingtu_slam_contract)
    add_test(NAME lingtu_slam_fastlio2_mock_flow COMMAND test_fastlio2_mock_flow)
    add_executable(test_initial_body_origin test_initial_body_origin.cpp)
    target_link_libraries(test_initial_body_origin PRIVATE lingtu_slam_contract)
    add_executable(test_icp_diagnostics test_icp_diagnostics.cpp)
    target_include_directories(test_icp_diagnostics PRIVATE
      "${CMAKE_CURRENT_SOURCE_DIR}/../../localizer/src/localizers")
    target_link_libraries(test_icp_diagnostics PRIVATE lingtu_slam_contract)
    add_test(NAME lingtu_slam_icp_diagnostics COMMAND test_icp_diagnostics)
    add_executable(test_imu_acceleration_scale
      test_imu_acceleration_scale.cpp
      "${FASTLIO2_DIR}/src/map_builder/commons.cpp"
      "${FASTLIO2_DIR}/src/map_builder/ieskf.cpp"
      "${FASTLIO2_DIR}/src/map_builder/imu_processor.cpp")
    target_include_directories(test_imu_acceleration_scale PRIVATE
      "${FASTLIO2_DIR}/src"
      "${FASTLIO2_DIR}/src/map_builder"
      ${PCL_INCLUDE_DIRS})
    target_link_libraries(test_imu_acceleration_scale PRIVATE
      Eigen3::Eigen
      Threads::Threads
      yaml-cpp
      ${PCL_LIBRARIES})
  endif()
endif()

if(LINGTU_SLAM_BUILD_PYTHON_BINDINGS)
  include(FetchContent)
  find_package(Python COMPONENTS Interpreter Development QUIET)

  if(Python_FOUND AND EXISTS "${Python_INCLUDE_DIRS}/Python.h")
    execute_process(
      COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
      OUTPUT_VARIABLE _nanobind_cmake_dir
      OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_QUIET
      RESULT_VARIABLE _nanobind_probe_result)
    if(_nanobind_probe_result EQUAL 0 AND EXISTS "${_nanobind_cmake_dir}")
      list(APPEND CMAKE_PREFIX_PATH "${_nanobind_cmake_dir}")
    endif()

    find_package(nanobind CONFIG QUIET)
    if(NOT nanobind_FOUND)
      FetchContent_Declare(nanobind
        GIT_REPOSITORY https://github.com/wjakob/nanobind.git
        GIT_TAG        v2.12.0)
      FetchContent_MakeAvailable(nanobind)
    endif()

    nanobind_add_module(_native bind.cpp)
    target_link_libraries(_native PRIVATE lingtu_slam_contract)
    target_compile_features(_native PRIVATE cxx_std_17)

    set(_SLAM_NATIVE_OUTPUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/..")
    set_target_properties(_native PROPERTIES
      LIBRARY_OUTPUT_DIRECTORY "${_SLAM_NATIVE_OUTPUT_DIR}"
      RUNTIME_OUTPUT_DIRECTORY "${_SLAM_NATIVE_OUTPUT_DIR}")
    foreach(_cfg DEBUG RELEASE RELWITHDEBINFO MINSIZEREL)
      set_target_properties(_native PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY_${_cfg} "${_SLAM_NATIVE_OUTPUT_DIR}"
        RUNTIME_OUTPUT_DIRECTORY_${_cfg} "${_SLAM_NATIVE_OUTPUT_DIR}")
    endforeach()
  else()
    message(WARNING "Skipping native SLAM Python bindings because Python development headers were not found")
  endif()
endif()

if(LINGTU_SLAM_BUILD_DDS_RUNTIME)
  find_package(iceoryx_binding_c QUIET)
  find_package(CycloneDDS REQUIRED)

  add_subdirectory(
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../message/cpp"
    "${CMAKE_CURRENT_BINARY_DIR}/message_cpp")

  find_program(CYCLONEDDS_IDLC_EXECUTABLE NAMES idlc REQUIRED)
  set(_LINGTU_SLAM_IDL "${CMAKE_CURRENT_SOURCE_DIR}/../../../message/idl/lingtu_slam.idl")
  set(_LINGTU_SLAM_IDL_HEADER "${CMAKE_CURRENT_BINARY_DIR}/lingtu_slam.h")
  set(_LINGTU_SLAM_IDL_SOURCE "${CMAKE_CURRENT_BINARY_DIR}/lingtu_slam.c")
  add_custom_command(
    OUTPUT "${_LINGTU_SLAM_IDL_HEADER}" "${_LINGTU_SLAM_IDL_SOURCE}"
    COMMAND "${CYCLONEDDS_IDLC_EXECUTABLE}"
            -l c
            "${_LINGTU_SLAM_IDL}"
    DEPENDS "${_LINGTU_SLAM_IDL}"
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
    VERBATIM)

  add_custom_target(lingtu_slam_cyclone_idl_generate
    DEPENDS "${_LINGTU_SLAM_IDL_HEADER}" "${_LINGTU_SLAM_IDL_SOURCE}")
  add_library(lingtu_slam_cyclone_idl STATIC "${_LINGTU_SLAM_IDL_SOURCE}")
  target_include_directories(lingtu_slam_cyclone_idl PUBLIC "${CMAKE_CURRENT_BINARY_DIR}")
  target_link_libraries(lingtu_slam_cyclone_idl PUBLIC CycloneDDS::ddsc)
  add_dependencies(lingtu_slam_cyclone_idl lingtu_slam_cyclone_idl_generate)

  add_executable(slamd cyclone_runtime.cpp)
  target_compile_features(slamd PRIVATE cxx_std_17)
  target_include_directories(slamd PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../../..")
  target_link_libraries(slamd
    lingtu_slam_contract
    lingtu_slam_cyclone_idl
    lingtu_dds_qos_profiles
    CycloneDDS::ddsc)

  add_executable(slamctl slam_control.cpp)
  target_compile_features(slamctl PRIVATE cxx_std_17)
  target_include_directories(slamctl PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../../..")
  target_link_libraries(slamctl
    lingtu_slam_cyclone_idl
    lingtu_dds_qos_profiles
    CycloneDDS::ddsc)
endif()
