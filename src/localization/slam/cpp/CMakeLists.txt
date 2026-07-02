cmake_minimum_required(VERSION 3.16)
project(lingtu_slam_contract LANGUAGES C CXX)

option(LINGTU_SLAM_BUILD_TESTS "Build native SLAM contract tests" ON)
option(LINGTU_SLAM_FASTLIO2_BACKEND "Build the real Fast-LIO2 algorithm backend" OFF)
option(LINGTU_SLAM_BUILD_PYTHON_BINDINGS "Build native SLAM Python bindings" OFF)
option(LINGTU_SLAM_BUILD_DDS_RUNTIME "Build ROS-free CycloneDDS C++ sensor-to-SLAM runtime" OFF)
option(LINGTU_SLAM_BUILD_CPP_DDS_RUNTIME "Deprecated alias for LINGTU_SLAM_BUILD_DDS_RUNTIME" OFF)
option(LINGTU_SLAM_BUILD_CYCLONE_DDS_RUNTIME "Deprecated alias for LINGTU_SLAM_BUILD_DDS_RUNTIME" OFF)
option(LINGTU_SLAM_BUILD_ROS2_DDS_RUNTIME "Build legacy ROS2/rclcpp DDS compatibility runtime" OFF)

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
  find_package(Eigen3 REQUIRED)
  find_package(PCL REQUIRED COMPONENTS common io filters kdtree)
  find_package(yaml-cpp REQUIRED)
  find_package(OpenMP QUIET)

  target_sources(lingtu_slam_contract PRIVATE
    "${FASTLIO2_DIR}/src/map_builder/commons.cpp"
    "${FASTLIO2_DIR}/src/map_builder/ieskf.cpp"
    "${FASTLIO2_DIR}/src/map_builder/imu_processor.cpp"
    "${FASTLIO2_DIR}/src/map_builder/ikd_Tree.cpp"
    "${FASTLIO2_DIR}/src/map_builder/lidar_processor.cpp"
    "${FASTLIO2_DIR}/src/map_builder/map_builder.cpp"
  )
  target_compile_definitions(lingtu_slam_contract PRIVATE
    LINGTU_HAS_FASTLIO2_BACKEND=1)
  target_include_directories(lingtu_slam_contract PRIVATE
    "${FASTLIO2_DIR}/src"
    "${FASTLIO2_DIR}/src/map_builder"
    ${PCL_INCLUDE_DIRS})
  target_link_libraries(lingtu_slam_contract PUBLIC
    Eigen3::Eigen
    yaml-cpp
    ${PCL_LIBRARIES})
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
  add_executable(test_slam_contract test_slam.cpp)
  target_link_libraries(test_slam_contract PRIVATE lingtu_slam_contract)
  if(LINGTU_SLAM_FASTLIO2_BACKEND)
    add_executable(test_fastlio2_mock_flow test_fastlio2_mock_flow.cpp)
    target_link_libraries(test_fastlio2_mock_flow PRIVATE lingtu_slam_contract)
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

if(LINGTU_SLAM_BUILD_ROS2_DDS_RUNTIME)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(livox_ros_driver2 REQUIRED)
  find_package(builtin_interfaces REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(std_msgs REQUIRED)

  add_executable(
    lingtu_slam_ros2_dds_runtime
    ../../adapters/ros2/cpp/ros2_dds_runtime.cpp)
  target_compile_features(lingtu_slam_ros2_dds_runtime PRIVATE cxx_std_17)
  target_include_directories(lingtu_slam_ros2_dds_runtime PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../..")
  target_link_libraries(lingtu_slam_ros2_dds_runtime lingtu_slam_contract)
  ament_target_dependencies(lingtu_slam_ros2_dds_runtime
    rclcpp
    livox_ros_driver2
    builtin_interfaces
    geometry_msgs
    sensor_msgs
    nav_msgs
    std_msgs)
endif()

if(LINGTU_SLAM_BUILD_DDS_RUNTIME)
  find_package(iceoryx_binding_c QUIET)
  find_package(CycloneDDS REQUIRED)

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

  add_executable(lingtu_slam_dds_runtime cyclone_runtime.cpp)
  target_compile_features(lingtu_slam_dds_runtime PRIVATE cxx_std_17)
  set_target_properties(lingtu_slam_dds_runtime PROPERTIES
    OUTPUT_NAME lingtu_slam_cyclone_runtime)
  target_include_directories(lingtu_slam_dds_runtime PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../../..")
  target_link_libraries(lingtu_slam_dds_runtime
    lingtu_slam_contract
    lingtu_slam_cyclone_idl
    CycloneDDS::ddsc)
  add_executable(lingtu_slam_cyclone_runtime ALIAS lingtu_slam_dds_runtime)

  add_executable(lingtu_slam_control slam_control.cpp)
  target_compile_features(lingtu_slam_control PRIVATE cxx_std_17)
  target_include_directories(lingtu_slam_control PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../../..")
  target_link_libraries(lingtu_slam_control
    lingtu_slam_cyclone_idl
    CycloneDDS::ddsc)
endif()
