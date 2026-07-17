cmake_minimum_required(VERSION 3.16)

# Header-only INTERFACE library: unified DDS QoS profiles and topic contracts.
# Consumers link this target to get dds_qos_profiles.hpp and dds_topics.hpp
# without needing to manage CycloneDDS include paths manually.

if(NOT TARGET lingtu_dds_qos_profiles)
  add_library(lingtu_dds_qos_profiles INTERFACE)
  target_include_directories(lingtu_dds_qos_profiles INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/../..")
  target_link_libraries(lingtu_dds_qos_profiles INTERFACE CycloneDDS::ddsc)
endif()
