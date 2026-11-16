include_guard(GLOBAL)

get_filename_component(_settings_repo "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
if(Python3_EXECUTABLE)
  set(_settings_python "${Python3_EXECUTABLE}")
else()
  find_program(_settings_python NAMES python python3
    PATHS "${_settings_repo}/.venv/Scripts" "${_settings_repo}/.venv/bin"
    NO_DEFAULT_PATH)
  if(NOT _settings_python)
    find_program(_settings_python NAMES python3 python)
  endif()
endif()
if(NOT _settings_python)
  message(FATAL_ERROR "Native settings generation requires Python with the project's PyYAML dependency")
endif()

set(_settings_source "${_settings_repo}/config/system/settings")
set(_settings_script "${_settings_repo}/scripts/build/settings.py")
set(_settings_include "${CMAKE_BINARY_DIR}/generated/settings")
set(_settings_header "${_settings_include}/lingtu/settings.hpp")
file(GLOB _settings_yaml CONFIGURE_DEPENDS "${_settings_source}/*.yaml")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  ${_settings_yaml} "${_settings_script}")
execute_process(
  COMMAND "${_settings_python}" "${_settings_script}" "${_settings_source}" "${_settings_header}"
  RESULT_VARIABLE _settings_result
  ERROR_VARIABLE _settings_error)
if(NOT _settings_result EQUAL 0)
  message(FATAL_ERROR "Cannot generate native settings: ${_settings_error}")
endif()

add_library(lingtu_settings INTERFACE)
target_include_directories(lingtu_settings INTERFACE "${_settings_include}")
target_compile_features(lingtu_settings INTERFACE cxx_std_17)
