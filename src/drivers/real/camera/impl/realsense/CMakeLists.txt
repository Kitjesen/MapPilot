cmake_minimum_required(VERSION 3.16)
project(lingtu_realsense_capture LANGUAGES CXX)
find_package(realsense2 CONFIG REQUIRED)
add_executable(realsense_capture capture.cpp)
target_compile_features(realsense_capture PRIVATE cxx_std_17)
target_link_libraries(realsense_capture PRIVATE realsense2::realsense2)
target_compile_options(realsense_capture PRIVATE -Wall -Wextra -Wpedantic)
install(TARGETS realsense_capture RUNTIME DESTINATION bin COMPONENT lingtu_runtime)
set_target_properties(realsense_capture PROPERTIES INSTALL_RPATH "$ORIGIN/../lib")
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  install(CODE
    "file(INSTALL DESTINATION \"\${CMAKE_INSTALL_PREFIX}/lib\"
      TYPE SHARED_LIBRARY FOLLOW_SYMLINK_CHAIN
      FILES \"$<TARGET_SONAME_FILE:realsense2::realsense2>\")"
    COMPONENT lingtu_runtime)
endif()
include(CTest)
if(BUILD_TESTING)
  add_executable(test_realsense_intrinsics
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../../../tests/drivers/realsense/test_intrinsics.cpp)
  target_include_directories(test_realsense_intrinsics PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  target_compile_features(test_realsense_intrinsics PRIVATE cxx_std_17)
  target_link_libraries(test_realsense_intrinsics PRIVATE realsense2::realsense2)
  add_test(NAME realsense_intrinsics COMMAND test_realsense_intrinsics)
  add_executable(test_realsense_frame_sequence
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../../../tests/drivers/realsense/test_frame_sequence.cpp)
  target_include_directories(test_realsense_frame_sequence PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  target_compile_features(test_realsense_frame_sequence PRIVATE cxx_std_17)
  add_test(NAME realsense_frame_sequence COMMAND test_realsense_frame_sequence)
endif()
