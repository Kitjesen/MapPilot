cmake_minimum_required(VERSION 3.16)
project(ota_daemon VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ──────────────── 依赖查找 ────────────────
# gRPC + Protobuf
find_package(Threads REQUIRED)
find_package(Protobuf REQUIRED)
find_package(gRPC CONFIG QUIET)

if(NOT gRPC_FOUND)
  # Fallback: 用 pkg-config 查找
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(GRPC REQUIRED grpc++ grpc)
  # 手动找插件
  find_program(GRPC_CPP_PLUGIN grpc_cpp_plugin)
else()
  # gRPC CONFIG 模式
  get_target_property(GRPC_CPP_PLUGIN gRPC::grpc_cpp_plugin LOCATION)
endif()

# OpenSSL (Ed25519 验签)
find_package(OpenSSL REQUIRED)

# yaml-cpp (配置文件)
find_package(yaml-cpp QUIET)
if(NOT yaml-cpp_FOUND)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(YAMLCPP REQUIRED yaml-cpp)
endif()

# CURL (DownloadFromUrl)
find_package(CURL REQUIRED)

# ──────────────── Proto 代码生成 ────────────────
set(PROTO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../robot_proto/proto")
set(PROTO_GEN_DIR "${CMAKE_CURRENT_BINARY_DIR}/proto_gen")
file(MAKE_DIRECTORY ${PROTO_GEN_DIR})

set(PROTO_FILES
  ${PROTO_DIR}/common.proto
  ${PROTO_DIR}/data.proto
)

set(PROTO_GEN_SRCS)
set(PROTO_GEN_HDRS)

foreach(PROTO_FILE ${PROTO_FILES})
  get_filename_component(PROTO_NAME ${PROTO_FILE} NAME_WE)

  set(PB_CC "${PROTO_GEN_DIR}/${PROTO_NAME}.pb.cc")
  set(PB_H  "${PROTO_GEN_DIR}/${PROTO_NAME}.pb.h")
  set(GRPC_CC "${PROTO_GEN_DIR}/${PROTO_NAME}.grpc.pb.cc")
  set(GRPC_H  "${PROTO_GEN_DIR}/${PROTO_NAME}.grpc.pb.h")

  add_custom_command(
    OUTPUT ${PB_CC} ${PB_H} ${GRPC_CC} ${GRPC_H}
    COMMAND protobuf::protoc
    ARGS
      --proto_path=${PROTO_DIR}
      --cpp_out=${PROTO_GEN_DIR}
      --grpc_out=${PROTO_GEN_DIR}
      --plugin=protoc-gen-grpc=${GRPC_CPP_PLUGIN}
      ${PROTO_FILE}
    DEPENDS ${PROTO_FILE}
    COMMENT "Generating proto stubs for ${PROTO_NAME}"
  )

  list(APPEND PROTO_GEN_SRCS ${PB_CC} ${GRPC_CC})
  list(APPEND PROTO_GEN_HDRS ${PB_H} ${GRPC_H})
endforeach()

# Proto 生成库 (静态)
add_library(ota_proto STATIC ${PROTO_GEN_SRCS} ${PROTO_GEN_HDRS})
target_include_directories(ota_proto PUBLIC ${PROTO_GEN_DIR})
target_link_libraries(ota_proto PUBLIC
  protobuf::libprotobuf
)
if(gRPC_FOUND)
  target_link_libraries(ota_proto PUBLIC gRPC::grpc++ gRPC::grpc)
else()
  target_include_directories(ota_proto PUBLIC ${GRPC_INCLUDE_DIRS})
  target_link_libraries(ota_proto PUBLIC ${GRPC_LIBRARIES})
endif()

# ──────────────── OTA Daemon 可执行文件 ────────────────
add_executable(ota_daemon
  src/main.cpp
  src/ota_service.cpp
  src/ota_config.cpp
  src/ota_file_ops.cpp
  src/ota_update.cpp
  src/ota_device.cpp
  src/utils.cpp
)

target_include_directories(ota_daemon PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${PROTO_GEN_DIR}
)

target_compile_definitions(ota_daemon PRIVATE
  OTA_DAEMON_VERSION="${PROJECT_VERSION}"
)

target_link_libraries(ota_daemon PRIVATE
  ota_proto
  Threads::Threads
  OpenSSL::SSL
  OpenSSL::Crypto
  CURL::libcurl
)

if(yaml-cpp_FOUND)
  # yaml-cpp 0.7 provides target "yaml-cpp" (no namespace),
  # yaml-cpp 0.8+ provides "yaml-cpp::yaml-cpp".
  if(TARGET yaml-cpp::yaml-cpp)
    target_link_libraries(ota_daemon PRIVATE yaml-cpp::yaml-cpp)
  else()
    target_link_libraries(ota_daemon PRIVATE yaml-cpp)
  endif()
else()
  target_include_directories(ota_daemon PRIVATE ${YAMLCPP_INCLUDE_DIRS})
  target_link_libraries(ota_daemon PRIVATE ${YAMLCPP_LIBRARIES})
endif()

# ──────────────── Install ────────────────
install(TARGETS ota_daemon RUNTIME DESTINATION bin)
install(FILES config/ota_daemon.yaml DESTINATION etc/ota_daemon)
install(FILES deploy/ota_daemon.service DESTINATION lib/systemd/system)
