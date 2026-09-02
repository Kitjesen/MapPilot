find_package(Threads REQUIRED)

function(lingtu_configure_inspection_test target)
  if(MSVC)
    target_compile_options(${target} PRIVATE /UNDEBUG)
  else()
    target_compile_options(${target} PRIVATE -UNDEBUG)
  endif()
  add_test(NAME ${target} COMMAND ${target})
endfunction()

add_executable(lingtu_inspection_test test_inspection.cpp)
target_link_libraries(lingtu_inspection_test PRIVATE
  lingtu_inspection_core Threads::Threads)
lingtu_configure_inspection_test(lingtu_inspection_test)

add_executable(lingtu_inspection_task_event_outbox_test
  "${CMAKE_CURRENT_LIST_DIR}/../endpoint/test_inspection_task_event_outbox.cpp")
target_link_libraries(lingtu_inspection_task_event_outbox_test PRIVATE
  lingtu_inspection_core)
lingtu_configure_inspection_test(lingtu_inspection_task_event_outbox_test)

add_executable(lingtu_inspection_c_api_exception_test
  test_c_api_exception.cpp
  "${LINGTU_INSPECTION_SOURCE_DIR}/c_api.cpp")
target_link_libraries(lingtu_inspection_c_api_exception_test PRIVATE
  lingtu_inspection_core)
target_compile_definitions(lingtu_inspection_c_api_exception_test PRIVATE
  LINGTU_INSPECTION_C_API_TEST_FORCE_STRING_ALLOCATION_FAILURE=1)
lingtu_configure_inspection_test(lingtu_inspection_c_api_exception_test)
