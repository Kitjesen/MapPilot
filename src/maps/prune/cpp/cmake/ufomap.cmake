include(FetchContent)
if(POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW)
endif()

FetchContent_Declare(ufomap
  URL "https://codeload.github.com/UnknownFreeOccupied/ufomap/tar.gz/fcb20f445d257280c039c07ab756c9d5b37ebd78")
FetchContent_GetProperties(ufomap)
if(NOT ufomap_POPULATED)
  FetchContent_Populate(ufomap)
endif()

# Guard x86-only includes while retaining upstream's portable bit operations on ARM.
set(ufomap_include "${CMAKE_CURRENT_BINARY_DIR}/ufomap-include")
foreach(header key.hpp code.hpp ray_caster/ray_caster.hpp octree/octree.hpp)
  file(READ "${ufomap_SOURCE_DIR}/include/ufo/map/${header}" content)
  string(REPLACE "#include <immintrin.h>"
    "#if defined(__BMI2__)\n#include <immintrin.h>\n#endif" content "${content}")
  # Offset signed coordinates before converting to an unsigned voxel key.
  string(REPLACE
    "std::floor(static_cast<node_size_t>(coord) / node_size_[0])) +\n\t\t       half_max_value_;"
    "std::floor(static_cast<node_size_t>(coord) / node_size_[0]) +\n\t\t       half_max_value_);"
    content "${content}")
  get_filename_component(header_dir "${ufomap_include}/ufo/map/${header}" DIRECTORY)
  file(MAKE_DIRECTORY "${header_dir}")
  file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/ufomap-header.tmp" "${content}")
  configure_file("${CMAKE_CURRENT_BINARY_DIR}/ufomap-header.tmp"
    "${ufomap_include}/ufo/map/${header}" COPYONLY)
endforeach()

find_package(OpenMP REQUIRED COMPONENTS CXX)
find_path(PRUNE_LZ4_INCLUDE_DIR NAMES lz4.h REQUIRED)
find_library(PRUNE_LZ4_LIBRARY NAMES lz4 REQUIRED)
find_path(PRUNE_LZF_INCLUDE_DIR NAMES liblzf/lzf.h REQUIRED)
find_library(PRUNE_LZF_LIBRARY NAMES lzf REQUIRED)

# Use a static target so the installed cleaner carries the pinned UFOMap implementation.
add_library(lingtu_prune_ufomap STATIC "${ufomap_SOURCE_DIR}/src/map/io.cpp")
target_include_directories(lingtu_prune_ufomap SYSTEM PUBLIC
  "${ufomap_include}"
  "${ufomap_SOURCE_DIR}/include"
  "${PRUNE_LZ4_INCLUDE_DIR}"
  "${PRUNE_LZF_INCLUDE_DIR}")
target_compile_features(lingtu_prune_ufomap PUBLIC cxx_std_20)
target_link_libraries(lingtu_prune_ufomap PUBLIC
  "${PRUNE_LZ4_LIBRARY}" "${PRUNE_LZF_LIBRARY}" OpenMP::OpenMP_CXX)

find_package(TBB QUIET)
if(TBB_FOUND)
  target_compile_definitions(lingtu_prune_ufomap PUBLIC UFO_PARALLEL=1)
  target_link_libraries(lingtu_prune_ufomap PUBLIC TBB::tbb)
endif()
