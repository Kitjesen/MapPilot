cmake_minimum_required(VERSION 3.16)
project(lingtu_inspection LANGUAGES CXX)

include(CTest)

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

if(MSVC)
  target_compile_options(lingtu_inspection_core PRIVATE /W4)
  target_compile_options(lingtu_inspection PRIVATE /W4)
else()
  target_compile_options(lingtu_inspection_core PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(lingtu_inspection PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(BUILD_TESTING)
  add_executable(lingtu_inspection_test test_inspection.cpp)
  target_link_libraries(lingtu_inspection_test PRIVATE lingtu_inspection_core)
  if(MSVC)
    target_compile_options(lingtu_inspection_test PRIVATE /UNDEBUG)
  else()
    target_compile_options(lingtu_inspection_test PRIVATE -UNDEBUG)
  endif()
  add_test(NAME lingtu_inspection_test COMMAND lingtu_inspection_test)

  add_executable(lingtu_inspection_c_api_exception_test
    test_c_api_exception.cpp
    c_api.cpp)
  target_link_libraries(lingtu_inspection_c_api_exception_test PRIVATE
    lingtu_inspection_core)
  target_compile_definitions(lingtu_inspection_c_api_exception_test PRIVATE
    LINGTU_INSPECTION_C_API_TEST_FORCE_STRING_ALLOCATION_FAILURE=1)
  if(MSVC)
    target_compile_options(lingtu_inspection_c_api_exception_test PRIVATE /UNDEBUG)
  else()
    target_compile_options(lingtu_inspection_c_api_exception_test PRIVATE -UNDEBUG)
  endif()
  add_test(
    NAME lingtu_inspection_c_api_exception_test
    COMMAND lingtu_inspection_c_api_exception_test)
endif()
