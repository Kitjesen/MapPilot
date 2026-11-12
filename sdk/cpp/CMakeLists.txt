cmake_minimum_required(VERSION 3.20)
project(LingTuSDK VERSION 2.3.0 LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
find_program(CARGO_EXECUTABLE cargo REQUIRED)

set(RUST_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../rust")
set(RUST_TARGET_DIR "${CMAKE_CURRENT_BINARY_DIR}/cargo")
set(RUST_RELEASE_DIR "${RUST_TARGET_DIR}/release")
if(WIN32)
    set(CORE_LIBRARY "${RUST_RELEASE_DIR}/lingtu_sdk.dll")
    set(CORE_IMPORT_LIBRARY "${RUST_RELEASE_DIR}/lingtu_sdk.dll.lib")
    set(CORE_OUTPUTS "${CORE_LIBRARY}" "${CORE_IMPORT_LIBRARY}")
elseif(APPLE)
    set(CORE_LIBRARY "${RUST_RELEASE_DIR}/liblingtu_sdk.dylib")
    set(CORE_OUTPUTS "${CORE_LIBRARY}")
else()
    set(CORE_LIBRARY "${RUST_RELEASE_DIR}/liblingtu_sdk.so")
    set(CORE_OUTPUTS "${CORE_LIBRARY}")
endif()

file(GLOB_RECURSE RUST_SOURCES CONFIGURE_DEPENDS "${RUST_SOURCE_DIR}/src/*.rs")
add_custom_command(
    OUTPUT ${CORE_OUTPUTS}
    COMMAND "${CMAKE_COMMAND}" -E env "CARGO_TARGET_DIR=${RUST_TARGET_DIR}"
            "${CARGO_EXECUTABLE}" build --manifest-path "${RUST_SOURCE_DIR}/Cargo.toml" --release --locked
    DEPENDS ${RUST_SOURCES} "${RUST_SOURCE_DIR}/Cargo.toml" "${RUST_SOURCE_DIR}/Cargo.lock"
    WORKING_DIRECTORY "${RUST_SOURCE_DIR}"
    COMMENT "Building the LingTu Rust communication core"
    VERBATIM
)
add_custom_target(lingtu_sdk_core_build DEPENDS ${CORE_OUTPUTS})
add_library(LingtuSdkCore SHARED IMPORTED GLOBAL)
set_target_properties(LingtuSdkCore PROPERTIES IMPORTED_LOCATION "${CORE_LIBRARY}")
if(WIN32)
    set_target_properties(LingtuSdkCore PROPERTIES IMPORTED_IMPLIB "${CORE_IMPORT_LIBRARY}")
elseif(NOT APPLE)
    set_target_properties(LingtuSdkCore PROPERTIES IMPORTED_NO_SONAME TRUE)
endif()
add_dependencies(LingtuSdkCore lingtu_sdk_core_build)

add_library(lingtu_sdk STATIC src/sdk.cpp)
add_library(LingTu::SDK ALIAS lingtu_sdk)
set_target_properties(lingtu_sdk PROPERTIES OUTPUT_NAME lingtu_sdk_cpp EXPORT_NAME SDK)
target_compile_features(lingtu_sdk PUBLIC cxx_std_17)
target_include_directories(lingtu_sdk PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)
target_link_libraries(lingtu_sdk PUBLIC LingtuSdkCore)
add_dependencies(lingtu_sdk lingtu_sdk_core_build)

if(CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)
    add_executable(lingtu_sdk_health examples/health.cpp)
    target_link_libraries(lingtu_sdk_health PRIVATE LingTu::SDK)
    include(CTest)
    set(SDK_EXECUTABLES lingtu_sdk_health)
    if(BUILD_TESTING)
        find_package(Python3 COMPONENTS Interpreter REQUIRED)
        add_executable(lingtu_sdk_probe tests/client_probe.cpp)
        target_link_libraries(lingtu_sdk_probe PRIVATE LingTu::SDK)
        list(APPEND SDK_EXECUTABLES lingtu_sdk_probe)
        add_test(NAME lingtu_sdk_http
            COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_client.py"
                    "$<TARGET_FILE:lingtu_sdk_probe>"
        )
        add_test(NAME lingtu_sdk_websocket
            COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_websocket.py"
                    "$<TARGET_FILE:lingtu_sdk_probe>"
        )
    endif()
    if(WIN32)
        foreach(executable IN LISTS SDK_EXECUTABLES)
            add_custom_command(TARGET ${executable} POST_BUILD
                COMMAND "${CMAKE_COMMAND}" -E copy_if_different
                        "$<TARGET_FILE:LingtuSdkCore>" "$<TARGET_FILE_DIR:${executable}>"
                VERBATIM
            )
        endforeach()
    endif()
endif()

install(TARGETS lingtu_sdk EXPORT LingTuSDKTargets ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}")
install(FILES include/lingtu/sdk.hpp DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/lingtu")
install(FILES README.md DESTINATION "${CMAKE_INSTALL_DOCDIR}")
if(WIN32)
    install(FILES "${CORE_LIBRARY}" DESTINATION "${CMAKE_INSTALL_BINDIR}")
    install(FILES "${CORE_IMPORT_LIBRARY}" DESTINATION "${CMAKE_INSTALL_LIBDIR}")
else()
    install(FILES "${CORE_LIBRARY}" DESTINATION "${CMAKE_INSTALL_LIBDIR}")
endif()

configure_package_config_file(cmake/LingTuSDKConfig.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/LingTuSDKConfig.cmake"
    INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/LingTuSDK"
    PATH_VARS CMAKE_INSTALL_BINDIR CMAKE_INSTALL_LIBDIR
)
write_basic_package_version_file("${CMAKE_CURRENT_BINARY_DIR}/LingTuSDKConfigVersion.cmake"
    VERSION "${PROJECT_VERSION}" COMPATIBILITY SameMajorVersion
)
install(EXPORT LingTuSDKTargets NAMESPACE LingTu:: DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/LingTuSDK")
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/LingTuSDKConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/LingTuSDKConfigVersion.cmake"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/LingTuSDK"
)

if(CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)
    set(CPACK_PACKAGE_NAME "lingtu-sdk-cpp")
    set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
    set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${PROJECT_VERSION}-${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}")
    if(WIN32)
        set(CPACK_GENERATOR "ZIP")
    else()
        set(CPACK_GENERATOR "TGZ")
    endif()
    include(CPack)
endif()
