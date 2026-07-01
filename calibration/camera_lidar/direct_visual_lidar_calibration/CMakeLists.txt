cmake_minimum_required(VERSION 3.16)
project(direct_visual_lidar_calibration)

add_compile_options(-std=c++17)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
set(LINGTU_CAMERA_LIDAR_USE_RUST_OPTIMIZER_DEFAULT ON)
option(
  LINGTU_CAMERA_LIDAR_USE_RUST_OPTIMIZER
  "Use LingTu Rust camera-LiDAR optimizer C ABI for CT-ICP/CT-GICP factor linearization and dynamic integration smoothing"
  ${LINGTU_CAMERA_LIDAR_USE_RUST_OPTIMIZER_DEFAULT}
)
set(_LINGTU_CAMERA_LIDAR_REQUIRE_RUST_DEFAULT OFF)
if(LINGTU_CAMERA_LIDAR_USE_RUST_OPTIMIZER)
  set(_LINGTU_CAMERA_LIDAR_REQUIRE_RUST_DEFAULT ON)
endif()
option(
  LINGTU_CAMERA_LIDAR_REQUIRE_RUST_OPTIMIZER
  "Fail dynamic integration instead of silently continuing without the Rust CT-GICP optimizer"
  ${_LINGTU_CAMERA_LIDAR_REQUIRE_RUST_DEFAULT}
)
if(LINGTU_CAMERA_LIDAR_REQUIRE_RUST_OPTIMIZER AND NOT LINGTU_CAMERA_LIDAR_USE_RUST_OPTIMIZER)
  message(
    FATAL_ERROR
    "LINGTU_CAMERA_LIDAR_REQUIRE_RUST_OPTIMIZER=ON requires "
    "LINGTU_CAMERA_LIDAR_USE_RUST_OPTIMIZER=ON."
  )
endif()

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

if($ENV{ROS_VERSION} EQUAL 1)
  # ROS1
  find_package(catkin REQUIRED COMPONENTS rosbag sensor_msgs cv_bridge)
  catkin_package()
else()
  # ROS2
  find_package(ament_cmake_auto REQUIRED)
  find_package(ament_cmake_python REQUIRED)
  ament_auto_find_build_dependencies()
endif()

find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options date_time)
find_package(Iridescence REQUIRED)
find_package(Threads REQUIRED)

set(OLD_DISTRO "humble;galactic;foxy")
if($ENV{ROS_DISTRO} IN_LIST OLD_DISTRO)
  add_definitions(-DCV_BRIDGE_INCLUDE_H)
else()
  add_definitions(-DCV_BRIDGE_INCLUDE_HPP)
endif()

find_package(OpenMP)
if (OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# direct_visual_lidar_calibration
add_library(direct_visual_lidar_calibration SHARED
  src/camera/create_camera.cpp
  src/vlcal/common/frame.cpp
  src/vlcal/common/frame_cpu.cpp
  src/vlcal/common/ivox.cpp
  src/vlcal/common/cloud_covariance_estimation.cpp
  src/vlcal/common/time_keeper.cpp
  src/vlcal/common/estimate_fov.cpp
  src/vlcal/common/estimate_pose.cpp
  src/vlcal/common/points_color_updater.cpp
  src/vlcal/common/visual_lidar_data.cpp
  src/vlcal/common/visual_lidar_visualizer.cpp
  src/vlcal/preprocess/preprocess.cpp
  src/vlcal/preprocess/generate_lidar_image.cpp
  src/vlcal/preprocess/static_point_cloud_integrator.cpp
  src/vlcal/preprocess/dynamic_point_cloud_integrator.cpp
  src/vlcal/calib/cost_calculator_nid.cpp
  src/vlcal/calib/view_culling.cpp
  src/vlcal/calib/visual_camera_calibration.cpp
)
target_include_directories(direct_visual_lidar_calibration PUBLIC
  include
  thirdparty/json/include
  thirdparty/Bonxai/include
  thirdparty/nanoflann/include
  thirdparty/Sophus
  ${Boost_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(direct_visual_lidar_calibration
  fmt
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
  ${CERES_LIBRARIES}
  ${OpenCV_LIBRARIES}
  Iridescence::Iridescence
)
target_compile_definitions(direct_visual_lidar_calibration PRIVATE LINGTU_CAMERA_LIDAR_GTSAM_FREE_BUILD)

if(LINGTU_CAMERA_LIDAR_USE_RUST_OPTIMIZER)
  set(_LINGTU_CAMERA_LIDAR_PREBUILT_OPTIMIZER_LIBRARY "")
  if(DEFINED ENV{LINGTU_CAMERA_LIDAR_OPTIMIZER_LIBRARY})
    set(_LINGTU_CAMERA_LIDAR_PREBUILT_OPTIMIZER_LIBRARY "$ENV{LINGTU_CAMERA_LIDAR_OPTIMIZER_LIBRARY}")
  elseif(DEFINED ENV{LINGTU_CAMERA_LIDAR_OPTIMIZER_LIB_DIR})
    find_library(
      _LINGTU_CAMERA_LIDAR_PREBUILT_OPTIMIZER_LIBRARY
      NAMES lingtu_camera_lidar_optimizer liblingtu_camera_lidar_optimizer
      HINTS "$ENV{LINGTU_CAMERA_LIDAR_OPTIMIZER_LIB_DIR}"
    )
  endif()
  set(
    LINGTU_CAMERA_LIDAR_OPTIMIZER_LIBRARY
    "${_LINGTU_CAMERA_LIDAR_PREBUILT_OPTIMIZER_LIBRARY}"
    CACHE FILEPATH
    "Optional prebuilt LingTu camera-LiDAR optimizer library. Leave empty to build with cargo."
  )
  if(LINGTU_CAMERA_LIDAR_OPTIMIZER_LIBRARY)
    add_library(lingtu_camera_lidar_optimizer UNKNOWN IMPORTED)
    set_target_properties(lingtu_camera_lidar_optimizer PROPERTIES
      IMPORTED_LOCATION "${LINGTU_CAMERA_LIDAR_OPTIMIZER_LIBRARY}"
    )
  else()
    find_program(CARGO_EXECUTABLE cargo REQUIRED)
    set(CAMERA_LIDAR_OPTIMIZER_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../../src/kernels/calibration/camera_lidar_optimizer")
    set(CAMERA_LIDAR_OPTIMIZER_MANIFEST "${CAMERA_LIDAR_OPTIMIZER_ROOT}/Cargo.toml")
    set(CAMERA_LIDAR_OPTIMIZER_TARGET_DIR "${CMAKE_CURRENT_BINARY_DIR}/camera_lidar_optimizer_cargo")
    if(WIN32)
      if(MSVC)
        set(CAMERA_LIDAR_OPTIMIZER_STATIC_LIB "${CAMERA_LIDAR_OPTIMIZER_TARGET_DIR}/release/lingtu_camera_lidar_optimizer.lib")
      else()
        set(CAMERA_LIDAR_OPTIMIZER_STATIC_LIB "${CAMERA_LIDAR_OPTIMIZER_TARGET_DIR}/release/liblingtu_camera_lidar_optimizer.a")
      endif()
    else()
      set(CAMERA_LIDAR_OPTIMIZER_STATIC_LIB "${CAMERA_LIDAR_OPTIMIZER_TARGET_DIR}/release/liblingtu_camera_lidar_optimizer.a")
    endif()

    file(GLOB_RECURSE CAMERA_LIDAR_OPTIMIZER_SOURCES CONFIGURE_DEPENDS
      "${CAMERA_LIDAR_OPTIMIZER_ROOT}/src/*.rs"
      "${CAMERA_LIDAR_OPTIMIZER_ROOT}/Cargo.lock"
    )

    add_custom_command(
      OUTPUT "${CAMERA_LIDAR_OPTIMIZER_STATIC_LIB}"
      COMMAND ${CMAKE_COMMAND} -E env "CARGO_TARGET_DIR=${CAMERA_LIDAR_OPTIMIZER_TARGET_DIR}" ${CARGO_EXECUTABLE} build --release --manifest-path "${CAMERA_LIDAR_OPTIMIZER_MANIFEST}"
      DEPENDS "${CAMERA_LIDAR_OPTIMIZER_MANIFEST}" ${CAMERA_LIDAR_OPTIMIZER_SOURCES}
      WORKING_DIRECTORY "${CAMERA_LIDAR_OPTIMIZER_ROOT}"
      VERBATIM
    )
    add_custom_target(camera_lidar_optimizer_rust DEPENDS "${CAMERA_LIDAR_OPTIMIZER_STATIC_LIB}")
    add_library(lingtu_camera_lidar_optimizer STATIC IMPORTED)
    set_target_properties(lingtu_camera_lidar_optimizer PROPERTIES
      IMPORTED_LOCATION "${CAMERA_LIDAR_OPTIMIZER_STATIC_LIB}"
    )
    add_dependencies(lingtu_camera_lidar_optimizer camera_lidar_optimizer_rust)
  endif()
  target_compile_definitions(direct_visual_lidar_calibration PRIVATE LINGTU_CAMERA_LIDAR_USE_RUST_OPTIMIZER)
  if(LINGTU_CAMERA_LIDAR_REQUIRE_RUST_OPTIMIZER)
    target_compile_definitions(direct_visual_lidar_calibration PRIVATE LINGTU_CAMERA_LIDAR_REQUIRE_RUST_OPTIMIZER)
  endif()
  target_link_libraries(direct_visual_lidar_calibration lingtu_camera_lidar_optimizer Threads::Threads ${CMAKE_DL_LIBS})
  if(UNIX AND NOT APPLE)
    target_link_libraries(direct_visual_lidar_calibration m)
  endif()
  if(WIN32)
    target_link_libraries(direct_visual_lidar_calibration kernel32 ntdll userenv ws2_32 dbghelp)
  endif()
endif()


if($ENV{ROS_VERSION} EQUAL 1)
  # ROS1
  add_executable(preprocess
    src/preprocess_ros1.cpp
  )
  target_link_libraries(preprocess
    direct_visual_lidar_calibration
  )
  target_include_directories(preprocess PUBLIC
    ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(preprocess
    ${catkin_LIBRARIES}
  )
else()
  # ROS2
  ament_auto_add_executable(preprocess
    src/preprocess_ros2.cpp
  )
  target_link_libraries(preprocess
    direct_visual_lidar_calibration
  )
endif()

# preprocess_map
add_executable(preprocess_map
  src/preprocess_map.cpp
)
target_include_directories(preprocess_map PRIVATE
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(preprocess_map
  direct_visual_lidar_calibration
)

# initial_guess_manual
add_executable(initial_guess_manual
  src/initial_guess_manual.cpp
)
target_include_directories(initial_guess_manual PRIVATE
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(initial_guess_manual
  direct_visual_lidar_calibration
)

# initial_guess_auto
add_executable(initial_guess_auto
  src/initial_guess_auto.cpp
)
target_include_directories(initial_guess_auto PRIVATE
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(initial_guess_auto
  direct_visual_lidar_calibration
)

# calibrate
add_executable(calibrate
  src/calibrate.cpp
)
target_include_directories(calibrate PRIVATE
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(calibrate
  direct_visual_lidar_calibration
)

# viewer
add_executable(viewer
  src/viewer.cpp
)
target_include_directories(viewer PRIVATE
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(viewer
  direct_visual_lidar_calibration
)

install(
  TARGETS
  direct_visual_lidar_calibration
)

install(
  TARGETS
  preprocess_map
  initial_guess_manual
  initial_guess_auto
  calibrate
  viewer
  DESTINATION
  lib/${PROJECT_NAME}
)

if($ENV{ROS_VERSION} EQUAL 1)
  # ROS1
else()
  # ROS2
  ament_python_install_package(${PROJECT_NAME})
  install(PROGRAMS scripts/find_matches_superglue.py DESTINATION lib/${PROJECT_NAME})
  ament_export_libraries(direct_visual_lidar_calibration)
  ament_auto_package()
endif()
