cmake_minimum_required(VERSION 3.16)
project(lingtu_yolo_trt LANGUAGES CXX)

find_path(TENSORRT_INCLUDE_DIR NvInfer.h
  PATHS /usr/include/aarch64-linux-gnu /usr/include)
find_library(TENSORRT_LIBRARY nvinfer
  PATHS /usr/lib/aarch64-linux-gnu /usr/lib)
find_library(CUDART_LIBRARY cudart
  PATHS
    /usr/local/cuda/lib64
    /usr/local/cuda/targets/aarch64-linux/lib
    /usr/lib/aarch64-linux-gnu)

if(NOT TENSORRT_INCLUDE_DIR OR NOT TENSORRT_LIBRARY OR NOT CUDART_LIBRARY)
  message(FATAL_ERROR "TensorRT headers, libnvinfer, and libcudart are required")
endif()

add_library(lingtu_yolo_trt SHARED yolo_trt.cpp)
target_compile_features(lingtu_yolo_trt PRIVATE cxx_std_17)
target_include_directories(lingtu_yolo_trt PRIVATE
  "${TENSORRT_INCLUDE_DIR}"
  /usr/local/cuda/include)
target_link_libraries(lingtu_yolo_trt PRIVATE
  "${TENSORRT_LIBRARY}"
  "${CUDART_LIBRARY}")
set_target_properties(lingtu_yolo_trt PROPERTIES OUTPUT_NAME lingtu_yolo_trt)
