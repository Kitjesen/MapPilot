cmake_minimum_required(VERSION 3.16)
project(lingtu_camera_video LANGUAGES CXX)

if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
  message(FATAL_ERROR "lingtu_camera_video consumes Linux POSIX camera SHM")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
find_package(Threads REQUIRED)

add_library(lingtu_video_encoder STATIC encoder.cpp)
target_include_directories(lingtu_video_encoder PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/..")
target_link_libraries(lingtu_video_encoder PUBLIC Threads::Threads rt)
target_compile_options(lingtu_video_encoder PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_executable(lingtu_camera_video main.cpp)
target_link_libraries(lingtu_camera_video PRIVATE lingtu_video_encoder)
target_compile_options(lingtu_camera_video PRIVATE -Wall -Wextra -Wpedantic -Werror)

find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
  pkg_check_modules(GST QUIET IMPORTED_TARGET
    gstreamer-1.0>=1.24 gstreamer-app-1.0>=1.24 gstreamer-video-1.0>=1.24)
endif()
if(GST_FOUND)
  add_library(lingtu_video_gst STATIC gst.cpp)
  target_include_directories(lingtu_video_gst PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/..")
  target_link_libraries(lingtu_video_gst PUBLIC PkgConfig::GST Threads::Threads rt)
  target_compile_options(lingtu_video_gst PRIVATE -Wall -Wextra -Wpedantic -Werror)
  add_executable(lingtu_camera_webrtc stream_main.cpp)
  target_link_libraries(lingtu_camera_webrtc PRIVATE lingtu_video_gst)
  target_compile_options(lingtu_camera_webrtc PRIVATE -Wall -Wextra -Wpedantic -Werror)
  install(TARGETS lingtu_camera_webrtc RUNTIME DESTINATION bin COMPONENT lingtu_runtime)
  set(LINGTU_VIDEO_RUNTIME_PREFIX "" CACHE PATH "Prepared target GStreamer runtime prefix")
  set(LINGTU_VIDEO_RUNTIME_MULTIARCH "" CACHE STRING "Prepared target runtime multiarch libdir")
  if(LINGTU_VIDEO_RUNTIME_PREFIX)
    if(NOT LINGTU_VIDEO_RUNTIME_MULTIARCH)
      message(FATAL_ERROR "LINGTU_VIDEO_RUNTIME_MULTIARCH is required with a runtime prefix")
    endif()
    foreach(runtime_file env.sh runtime-arch)
      if(NOT EXISTS "${LINGTU_VIDEO_RUNTIME_PREFIX}/${runtime_file}")
        message(FATAL_ERROR "Prepared camera-video runtime is missing ${runtime_file}")
      endif()
    endforeach()
    if(NOT IS_DIRECTORY "${LINGTU_VIDEO_RUNTIME_PREFIX}/lib/${LINGTU_VIDEO_RUNTIME_MULTIARCH}")
      message(FATAL_ERROR "Prepared camera-video runtime multiarch libdir is missing")
    endif()
    if(NOT EXISTS "${LINGTU_VIDEO_RUNTIME_PREFIX}/libexec/gstreamer-1.0/gst-plugin-scanner")
      message(FATAL_ERROR "Prepared camera-video runtime plugin scanner is missing")
    endif()
    install(FILES
      "${LINGTU_VIDEO_RUNTIME_PREFIX}/env.sh"
      "${LINGTU_VIDEO_RUNTIME_PREFIX}/runtime-arch"
      DESTINATION lib/lingtu/video
      COMPONENT lingtu_runtime)
    install(DIRECTORY "${LINGTU_VIDEO_RUNTIME_PREFIX}/lib/"
      DESTINATION lib/lingtu/video/lib
      USE_SOURCE_PERMISSIONS
      COMPONENT lingtu_runtime)
    install(DIRECTORY "${LINGTU_VIDEO_RUNTIME_PREFIX}/libexec/"
      DESTINATION lib/lingtu/video/libexec
      USE_SOURCE_PERMISSIONS
      COMPONENT lingtu_runtime)
    install(DIRECTORY "${LINGTU_VIDEO_RUNTIME_PREFIX}/bin/"
      DESTINATION lib/lingtu/video/bin
      USE_SOURCE_PERMISSIONS
      COMPONENT lingtu_runtime)
  endif()
else()
  message(STATUS "GStreamer development libraries unavailable: only the FFmpeg reference is built")
endif()

include(CTest)
if(BUILD_TESTING)
  set(tests "${CMAKE_CURRENT_SOURCE_DIR}/../../../../../../tests/drivers/cpp/camera/native")
  foreach(test shm_reader video_encoder)
    add_executable(test_${test} "${tests}/test_${test}.cpp")
    target_link_libraries(test_${test} PRIVATE lingtu_video_encoder)
    target_compile_options(test_${test} PRIVATE -Wall -Wextra -Wpedantic -Werror -UNDEBUG)
    add_test(NAME camera_${test} COMMAND test_${test})
    set_tests_properties(camera_${test} PROPERTIES TIMEOUT 20)
  endforeach()
  add_executable(video_fixture "${tests}/video_fixture.cpp")
  target_include_directories(video_fixture PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/..")
  target_link_libraries(video_fixture PRIVATE Threads::Threads rt)
  target_compile_options(video_fixture PRIVATE -Wall -Wextra -Wpedantic -Werror)
  find_package(Python3 COMPONENTS Interpreter REQUIRED)
  add_test(NAME camera_video_cli COMMAND "${Python3_EXECUTABLE}"
    "${tests}/test_video_cli.py" --video "$<TARGET_FILE:lingtu_camera_video>"
    --fixture "$<TARGET_FILE:video_fixture>"
    --artifacts "${CMAKE_CURRENT_BINARY_DIR}/evidence")
  set_tests_properties(camera_video_cli PROPERTIES TIMEOUT 30)
  if(GST_FOUND)
    add_executable(test_video_clock "${tests}/test_video_clock.cpp")
    target_link_libraries(test_video_clock PRIVATE lingtu_video_gst)
    target_compile_options(test_video_clock PRIVATE -Wall -Wextra -Wpedantic -Werror -UNDEBUG)
    add_test(NAME camera_video_clock COMMAND test_video_clock)
    set_tests_properties(camera_video_clock PROPERTIES TIMEOUT 5)
    add_test(NAME camera_video_whep_clock_cli COMMAND "${Python3_EXECUTABLE}"
      "${tests}/test_video_whep_clock_cli.py" --video "$<TARGET_FILE:lingtu_camera_webrtc>"
      --fixture "$<TARGET_FILE:video_fixture>"
      --artifacts "${CMAKE_CURRENT_BINARY_DIR}/evidence")
    set_tests_properties(camera_video_whep_clock_cli PROPERTIES TIMEOUT 20 SKIP_RETURN_CODE 77)
    add_executable(test_video_gst "${tests}/test_video_gst.cpp")
    target_link_libraries(test_video_gst PRIVATE lingtu_video_gst)
    target_compile_options(test_video_gst PRIVATE -Wall -Wextra -Wpedantic -Werror -UNDEBUG)
    add_test(NAME camera_video_gstreamer COMMAND test_video_gst)
    set_tests_properties(camera_video_gstreamer PROPERTIES TIMEOUT 20)
  endif()
  find_program(LINGTU_VIDEO_FFMPEG NAMES ffmpeg)
  if(LINGTU_VIDEO_FFMPEG)
    add_test(NAME camera_video_h264 COMMAND "${Python3_EXECUTABLE}"
      "${tests}/test_video_cli.py" --video "$<TARGET_FILE:lingtu_camera_video>"
      --fixture "$<TARGET_FILE:video_fixture>" --ffmpeg "${LINGTU_VIDEO_FFMPEG}"
      --artifacts "${CMAKE_CURRENT_BINARY_DIR}/evidence")
    set_tests_properties(camera_video_h264 PROPERTIES TIMEOUT 45)
    if(GST_FOUND)
      add_test(NAME camera_video_gstreamer_cli COMMAND "${Python3_EXECUTABLE}"
        "${tests}/test_video_gst_cli.py" --video "$<TARGET_FILE:lingtu_camera_webrtc>"
        --fixture "$<TARGET_FILE:video_fixture>" --ffmpeg "${LINGTU_VIDEO_FFMPEG}"
        --artifacts "${CMAKE_CURRENT_BINARY_DIR}/evidence")
      set_tests_properties(camera_video_gstreamer_cli PROPERTIES TIMEOUT 45)
    endif()
  endif()
endif()
