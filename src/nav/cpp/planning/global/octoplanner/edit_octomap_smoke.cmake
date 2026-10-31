if(NOT DEFINED MAKE_TEST)
  message(FATAL_ERROR "MAKE_TEST is required")
endif()
if(NOT DEFINED EDITOR)
  message(FATAL_ERROR "EDITOR is required")
endif()
if(NOT DEFINED WORK_DIR)
  message(FATAL_ERROR "WORK_DIR is required")
endif()

file(MAKE_DIRECTORY "${WORK_DIR}")
set(BT_PATH "${WORK_DIR}/source.bt")
set(OT_PATH "${WORK_DIR}/binary_named_as_ot.ot")
set(OUT_PATH "${WORK_DIR}/edited.ot")

execute_process(
  COMMAND "${MAKE_TEST}" --output "${BT_PATH}"
  RESULT_VARIABLE make_result
  OUTPUT_VARIABLE make_stdout
  ERROR_VARIABLE make_stderr
)
if(NOT make_result EQUAL 0)
  message(FATAL_ERROR "make test octomap failed: ${make_stdout}\n${make_stderr}")
endif()

configure_file("${BT_PATH}" "${OT_PATH}" COPYONLY)

execute_process(
  COMMAND "${EDITOR}"
    --map "${OT_PATH}"
    --output "${OUT_PATH}"
    --state traversable
    --x 0
    --y 0
    --z 0
    --radius 0.5
  RESULT_VARIABLE edit_result
  OUTPUT_VARIABLE edit_stdout
  ERROR_VARIABLE edit_stderr
)
if(NOT edit_result EQUAL 0)
  message(FATAL_ERROR "edit octomap failed: ${edit_stdout}\n${edit_stderr}")
endif()
if(NOT EXISTS "${OUT_PATH}")
  message(FATAL_ERROR "editor did not create ${OUT_PATH}")
endif()
file(SIZE "${OUT_PATH}" out_size)
if(out_size LESS_EQUAL 0)
  message(FATAL_ERROR "editor wrote an empty output: ${OUT_PATH}")
endif()

# At 0.2 m resolution this box covers 11 keys along each axis. Floating
# coordinate sampling previously skipped layers at these exact boundaries.
execute_process(
  COMMAND "${EDITOR}" --map "${BT_PATH}" --output "${WORK_DIR}/box.ot"
    --state free --shape box --x 10 --y 0 --z 0.4 --radius 1
  RESULT_VARIABLE box_result
  OUTPUT_VARIABLE box_stdout
  ERROR_VARIABLE box_stderr
)
if(NOT box_result EQUAL 0 OR NOT box_stdout MATCHES "\"edited_voxels\":1331")
  message(FATAL_ERROR "box edit skipped voxel keys: ${box_stdout}\n${box_stderr}")
endif()
