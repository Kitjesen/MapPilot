cmake_minimum_required(VERSION 3.16)
project(lingtu_inspection LANGUAGES CXX)

include(CTest)
include(GNUInstallDirs)

set(_LINGTU_SRC_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../..")

add_library(lingtu_inspection_core STATIC
  inspection.cpp
  store.cpp)
target_include_directories(lingtu_inspection_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${_LINGTU_SRC_ROOT})
target_compile_features(lingtu_inspection_core PUBLIC cxx_std_17)
set_target_properties(lingtu_inspection_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(lingtu_inspection SHARED c_api.cpp)
target_link_libraries(lingtu_inspection PRIVATE lingtu_inspection_core)
target_include_directories(lingtu_inspection PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_features(lingtu_inspection PUBLIC cxx_std_17)

install(TARGETS lingtu_inspection
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    COMPONENT lingtu_runtime
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    COMPONENT lingtu_runtime)

if(MSVC)
  target_compile_options(lingtu_inspection_core PRIVATE /W4)
  target_compile_options(lingtu_inspection PRIVATE /W4)
else()
  target_compile_options(lingtu_inspection_core PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(lingtu_inspection PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(BUILD_TESTING)
  set(LINGTU_INSPECTION_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
  add_subdirectory(
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../tests/nav/cpp/inspection"
    "${CMAKE_CURRENT_BINARY_DIR}/tests")
endif()
