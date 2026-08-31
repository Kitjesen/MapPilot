cmake_minimum_required(VERSION 3.16)
project(lingtu_explore_cpp LANGUAGES C CXX)

option(LINGTU_EXPLORE_CPP_BUILD_TESTS "Build LingTu explore C++ tests" OFF)

add_library(lingtu_explore_core
  frontier.cpp
  keypose_graph.cpp
  tare_policy.cpp)

target_compile_features(lingtu_explore_core PUBLIC cxx_std_17)
target_include_directories(lingtu_explore_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include/explore/cpp>)
target_compile_options(lingtu_explore_core PUBLIC
  $<$<CXX_COMPILER_ID:MSVC>:/utf-8>)
# GCC 10.1 changed the C++17 ABI for std::pair / std::tuple return values
# (mandatory copy-elision).  Suppress the per-TU notice; it is informational
# only and does not indicate a code defect.
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  target_compile_options(lingtu_explore_core PRIVATE -Wno-psabi)
endif()

# ── idlc-generated C type bindings (optional) ──────────────────────────
# The build system generates the shared LingTu IDL into this directory.
set(LINGTU_DDS_GEN_DIR "${CMAKE_CURRENT_BINARY_DIR}/dds_generated"
  CACHE PATH "Directory containing generated Explore DDS C bindings")

if(EXISTS "${LINGTU_DDS_GEN_DIR}/messages.c")
  message(STATUS "idlc generated source: messages.c")
  target_sources(lingtu_explore_core PRIVATE "${LINGTU_DDS_GEN_DIR}/messages.c")
  target_include_directories(lingtu_explore_core PUBLIC
    $<BUILD_INTERFACE:${LINGTU_DDS_GEN_DIR}>)
else()
  message(STATUS "idlc generated messages.c not found in ${LINGTU_DDS_GEN_DIR} (DDS types unavailable)")
endif()

# ── Optional CycloneDDS transport ────────────────────────────────────────
# When ON and CycloneDDS is found, defines LINGTU_EXPLORE_HAS_DDS which
# enables TareDdsTransport in tare_dds.hpp and its nanobind bindings.
option(LINGTU_EXPLORE_CPP_WITH_DDS
  "Enable native CycloneDDS transport for TARE exploration" OFF)

if(LINGTU_EXPLORE_CPP_WITH_DDS)
  find_package(CycloneDDS QUIET)
  if(CycloneDDS_FOUND)
    message(STATUS "CycloneDDS found: ${CycloneDDS_DIR}")
    target_compile_definitions(lingtu_explore_core PUBLIC LINGTU_EXPLORE_HAS_DDS)
    target_link_libraries(lingtu_explore_core PUBLIC CycloneDDS::ddsc)
    # The repo root contains message/cpp/ (topics.hpp, qos.hpp).
    target_include_directories(lingtu_explore_core PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../..>)
  else()
    message(WARNING
      "LINGTU_EXPLORE_CPP_WITH_DDS=ON but CycloneDDS not found; "
      "DDS transport will not be compiled.")
  endif()
endif()

if(LINGTU_EXPLORE_CPP_BUILD_TESTS)
  enable_testing()
  add_executable(test_tare_policy
    test_tare_policy.cpp)
  target_link_libraries(test_tare_policy PRIVATE lingtu_explore_core)
  add_test(NAME test_tare_policy COMMAND test_tare_policy)
endif()

# nanobind Python extension: exposes the C++ TarePolicy as lingtu_explore_kernel.
# Mirrors the optional Python-binding mode in src/nav/cpp/CMakeLists.txt. Off by default; built by
# scripts/build/build_explore_kernel.sh or by passing the option below.
option(LINGTU_EXPLORE_CPP_BUILD_PYTHON_BINDINGS
  "Build lingtu_explore_kernel Python bindings" OFF)
if(LINGTU_EXPLORE_CPP_BUILD_PYTHON_BINDINGS)
  find_package(Python COMPONENTS Interpreter Development QUIET)
endif()
if(LINGTU_EXPLORE_CPP_BUILD_PYTHON_BINDINGS AND Python_FOUND AND EXISTS "${Python_INCLUDE_DIRS}/Python.h")
  include(FetchContent)
  # nanobind replaces pybind11 for faster builds, smaller modules, and numpy zero-copy support.
  FetchContent_Declare(nanobind
    GIT_REPOSITORY https://github.com/wjakob/nanobind.git
    GIT_TAG        v2.12.0)
  FetchContent_MakeAvailable(nanobind)
  set(LINGTU_EXPLORE_BINDING_SOURCES
    bindings/bindings.cpp
    bindings/bind_types.cpp
    bindings/bind_tare.cpp)
  nanobind_add_module(lingtu_explore_kernel ${LINGTU_EXPLORE_BINDING_SOURCES})
  target_include_directories(lingtu_explore_kernel PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  target_link_libraries(lingtu_explore_kernel PRIVATE lingtu_explore_core)
  # Propagate DDS availability to bindings so bind_tare.cpp can conditionally
  # register methods or raise RuntimeError.
  if(LINGTU_EXPLORE_CPP_WITH_DDS AND CycloneDDS_FOUND)
    target_compile_definitions(lingtu_explore_kernel PRIVATE LINGTU_EXPLORE_HAS_DDS)
  endif()
elseif(LINGTU_EXPLORE_CPP_BUILD_PYTHON_BINDINGS)
  message(WARNING "Skipping lingtu_explore_kernel Python bindings because Python development headers were not found")
endif()

install(FILES
  explore_contract.hpp
  tare_policy.hpp
  DESTINATION include/explore/cpp)
