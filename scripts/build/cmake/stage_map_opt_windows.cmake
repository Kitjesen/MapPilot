cmake_minimum_required(VERSION 3.21)
# The executable's PE imports, not the caller's PATH, define its DLL closure.
set(CMAKE_GET_RUNTIME_DEPENDENCIES_PLATFORM "windows+pe")
set(CMAKE_GET_RUNTIME_DEPENDENCIES_TOOL "dumpbin")
set(CMAKE_GET_RUNTIME_DEPENDENCIES_COMMAND "${DUMPBIN}")
file(GET_RUNTIME_DEPENDENCIES
  EXECUTABLES "${TOOL_DIRECTORY}/lt_pgo.exe" "${TOOL_DIRECTORY}/lt_loop_verify.exe"
              "${TOOL_DIRECTORY}/pgo_test.exe" "${TOOL_DIRECTORY}/loop_information_test.exe"
              "${TOOL_DIRECTORY}/constraint_assembly_test.exe" "${TOOL_DIRECTORY}/online_graph_test.exe"
  DIRECTORIES "${DEPENDENCY_DIRECTORY}"
  PRE_EXCLUDE_REGEXES "[Aa][Pp][Ii]-[Mm][Ss]-.*" "[Ee][Xx][Tt]-[Mm][Ss]-.*"
  POST_EXCLUDE_REGEXES [=[[Cc]:[/\\][Ww][Ii][Nn][Dd][Oo][Ww][Ss][/\\][Ss][Yy][Ss][Tt][Ee][Mm]32[/\\].*]=]
  RESOLVED_DEPENDENCIES_VAR resolved
  UNRESOLVED_DEPENDENCIES_VAR missing)
if(missing)
  message(FATAL_ERROR "Unresolved map optimizer DLLs: ${missing}")
endif()
foreach(dependency IN LISTS resolved)
  file(COPY "${dependency}" DESTINATION "${TOOL_DIRECTORY}")
endforeach()
list(SORT resolved)
list(JOIN resolved "\n" dependency_lines)
file(WRITE "${TOOL_DIRECTORY}/runtime-dependencies.txt" "${dependency_lines}\n")
message(STATUS "Staged map optimizer runtime DLLs: ${resolved}")
