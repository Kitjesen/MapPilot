#define PY_SSIZE_T_CLEAN
#include <Python.h>

#include "octoplanner3d_core.hpp"

#include <exception>
#include <stdexcept>
#include <string>
#include <vector>

namespace octo = octoplanner3d::runtime;

namespace {

std::string stringFromDict(PyObject * payload, const char * key)
{
  PyObject * item = PyDict_GetItemString(payload, key);
  if (item == nullptr) {
    throw std::runtime_error(std::string("missing string field: ") + key);
  }
  const char * value = PyUnicode_AsUTF8(item);
  if (value == nullptr) {
    PyErr_Clear();
    throw std::runtime_error(std::string("field must be a string: ") + key);
  }
  return value;
}

std::vector<double> numberArrayFromDict(PyObject * payload, const char * key)
{
  PyObject * item = PyDict_GetItemString(payload, key);
  if (item == nullptr) {
    throw std::runtime_error(std::string("missing numeric array field: ") + key);
  }
  PyObject * seq = PySequence_Fast(item, "expected a numeric sequence");
  if (seq == nullptr) {
    PyErr_Clear();
    throw std::runtime_error(std::string("field must be a numeric array: ") + key);
  }

  std::vector<double> values;
  const Py_ssize_t size = PySequence_Fast_GET_SIZE(seq);
  values.reserve(static_cast<std::size_t>(size));
  PyObject ** items = PySequence_Fast_ITEMS(seq);
  for (Py_ssize_t i = 0; i < size; ++i) {
    const double value = PyFloat_AsDouble(items[i]);
    if (PyErr_Occurred()) {
      PyErr_Clear();
      Py_DECREF(seq);
      throw std::runtime_error(std::string("field contains non-numeric value: ") + key);
    }
    values.push_back(value);
  }
  Py_DECREF(seq);
  return values;
}

octo::Point pointFromVector(const std::vector<double> & values, const std::string & field)
{
  if (values.size() < 2) {
    throw std::runtime_error(field + " must contain at least x and y");
  }
  return octo::Point{values[0], values[1], values.size() >= 3 ? values[2] : 0.0};
}

PyObject * optionalOptionsDict(PyObject * payload)
{
  PyObject * options = PyDict_GetItemString(payload, "options");
  if (options == nullptr || options == Py_None) {
    return nullptr;
  }
  if (!PyDict_Check(options)) {
    throw std::runtime_error("options must be a dict");
  }
  return options;
}

double optionalDoubleFromDict(
  PyObject * options,
  const char * key,
  double default_value)
{
  if (options == nullptr) {
    return default_value;
  }
  PyObject * item = PyDict_GetItemString(options, key);
  if (item == nullptr || item == Py_None) {
    return default_value;
  }
  const double value = PyFloat_AsDouble(item);
  if (PyErr_Occurred()) {
    PyErr_Clear();
    throw std::runtime_error(std::string("option must be numeric: ") + key);
  }
  return value;
}

int optionalIntFromDict(PyObject * options, const char * key, int default_value)
{
  if (options == nullptr) {
    return default_value;
  }
  PyObject * item = PyDict_GetItemString(options, key);
  if (item == nullptr || item == Py_None) {
    return default_value;
  }
  const long value = PyLong_AsLong(item);
  if (PyErr_Occurred()) {
    PyErr_Clear();
    throw std::runtime_error(std::string("option must be integer: ") + key);
  }
  return static_cast<int>(value);
}

bool optionalBoolFromDict(PyObject * options, const char * key, bool default_value)
{
  if (options == nullptr) {
    return default_value;
  }
  PyObject * item = PyDict_GetItemString(options, key);
  if (item == nullptr || item == Py_None) {
    return default_value;
  }
  const int truthy = PyObject_IsTrue(item);
  if (truthy < 0) {
    PyErr_Clear();
    throw std::runtime_error(std::string("option must be boolean: ") + key);
  }
  return truthy != 0;
}

void applyOptionsFromPayload(octo::PlannerOptions & planner_options, PyObject * payload)
{
  PyObject * options = optionalOptionsDict(payload);
  planner_options.robot_radius = optionalDoubleFromDict(
    options,
    "robot_radius",
    planner_options.robot_radius);
  planner_options.max_iterations = optionalIntFromDict(
    options,
    "max_iterations",
    planner_options.max_iterations);
  planner_options.snap_search_radius_cells = optionalIntFromDict(
    options,
    "snap_search_radius_cells",
    planner_options.snap_search_radius_cells);
  planner_options.require_ground_support = optionalBoolFromDict(
    options,
    "require_ground_support",
    planner_options.require_ground_support);
  planner_options.strict_direct_ground_support = optionalBoolFromDict(
    options,
    "strict_direct_ground_support",
    planner_options.strict_direct_ground_support);
  planner_options.ground_support_xy_radius_cells = optionalIntFromDict(
    options,
    "ground_support_xy_radius_cells",
    planner_options.ground_support_xy_radius_cells);
  planner_options.ground_support_depth_cells = optionalIntFromDict(
    options,
    "ground_support_depth_cells",
    planner_options.ground_support_depth_cells);
  planner_options.enable_preblocked_costmap = optionalBoolFromDict(
    options,
    "enable_preblocked_costmap",
    planner_options.enable_preblocked_costmap);
  planner_options.preblocked_costmap_radius_cells = optionalIntFromDict(
    options,
    "preblocked_costmap_radius_cells",
    planner_options.preblocked_costmap_radius_cells);
  planner_options.preblocked_costmap_weight = optionalDoubleFromDict(
    options,
    "preblocked_costmap_weight",
    planner_options.preblocked_costmap_weight);
  planner_options.lowest_traversable_only = optionalBoolFromDict(
    options,
    "lowest_traversable_only",
    planner_options.lowest_traversable_only);
}

octo::PlanRequest requestFromPayload(PyObject * payload)
{
  octo::PlanRequest request;
  request.map_path = stringFromDict(payload, "map_path");
  request.start = pointFromVector(numberArrayFromDict(payload, "start"), "start");
  request.goal = pointFromVector(numberArrayFromDict(payload, "goal"), "goal");
  applyOptionsFromPayload(request.options, payload);
  return request;
}

void setItem(PyObject * dict, const char * key, PyObject * value)
{
  if (value == nullptr) {
    throw std::runtime_error(std::string("failed to create Python value for: ") + key);
  }
  if (PyDict_SetItemString(dict, key, value) != 0) {
    Py_DECREF(value);
    throw std::runtime_error(std::string("failed to set Python dict key: ") + key);
  }
  Py_DECREF(value);
}

PyObject * constraintsFromOptions(const octo::PlannerOptions & options)
{
  PyObject * constraints = PyDict_New();
  if (constraints == nullptr) {
    throw std::runtime_error("failed to allocate constraints dict");
  }
  try {
    setItem(constraints, "robot_radius", PyFloat_FromDouble(options.robot_radius));
    setItem(constraints, "max_iterations", PyLong_FromLong(options.max_iterations));
    setItem(
      constraints,
      "snap_search_radius_cells",
      PyLong_FromLong(options.snap_search_radius_cells));
    setItem(
      constraints,
      "require_ground_support",
      PyBool_FromLong(options.require_ground_support ? 1 : 0));
    setItem(
      constraints,
      "strict_direct_ground_support",
      PyBool_FromLong(options.strict_direct_ground_support ? 1 : 0));
    setItem(
      constraints,
      "ground_support_xy_radius_cells",
      PyLong_FromLong(options.ground_support_xy_radius_cells));
    setItem(
      constraints,
      "ground_support_depth_cells",
      PyLong_FromLong(options.ground_support_depth_cells));
    setItem(
      constraints,
      "enable_preblocked_costmap",
      PyBool_FromLong(options.enable_preblocked_costmap ? 1 : 0));
    setItem(
      constraints,
      "preblocked_costmap_radius_cells",
      PyLong_FromLong(options.preblocked_costmap_radius_cells));
    setItem(
      constraints,
      "preblocked_costmap_weight",
      PyFloat_FromDouble(options.preblocked_costmap_weight));
    setItem(
      constraints,
      "lowest_traversable_only",
      PyBool_FromLong(options.lowest_traversable_only ? 1 : 0));
  } catch (...) {
    Py_DECREF(constraints);
    throw;
  }
  return constraints;
}

PyObject * diagnosticsFromResult(const octo::PlanResult & result)
{
  PyObject * diagnostics = PyDict_New();
  if (diagnostics == nullptr) {
    throw std::runtime_error("failed to allocate diagnostics dict");
  }
  setItem(diagnostics, "source", PyUnicode_FromString("octoplanner3d_cpython"));
  setItem(diagnostics, "ros2_required", PyBool_FromLong(0));
  setItem(diagnostics, "pcd_conversion", PyBool_FromLong(result.pcd_conversion ? 1 : 0));
  setItem(diagnostics, "octomap_file", PyBool_FromLong(result.octomap_file ? 1 : 0));
  setItem(diagnostics, "path_points", PyLong_FromSize_t(result.path.size()));
  setItem(diagnostics, "goal_error_m", PyFloat_FromDouble(result.goal_error_m));
  setItem(diagnostics, "elapsed_ms", PyFloat_FromDouble(result.elapsed_ms));
  setItem(diagnostics, "constraints", constraintsFromOptions(result.options));
  return diagnostics;
}

PyObject * diagnosticsForError()
{
  PyObject * diagnostics = PyDict_New();
  if (diagnostics == nullptr) {
    throw std::runtime_error("failed to allocate diagnostics dict");
  }
  setItem(diagnostics, "source", PyUnicode_FromString("octoplanner3d_cpython"));
  setItem(diagnostics, "ros2_required", PyBool_FromLong(0));
  setItem(diagnostics, "pcd_conversion", PyBool_FromLong(octo::pcdConversionEnabled() ? 1 : 0));
  setItem(diagnostics, "octomap_file", PyBool_FromLong(1));
  setItem(diagnostics, "constraints", constraintsFromOptions(octo::PlannerOptions{}));
  return diagnostics;
}

PyObject * pathFromResult(const octo::PlanResult & result)
{
  PyObject * path = PyList_New(static_cast<Py_ssize_t>(result.path.size()));
  if (path == nullptr) {
    throw std::runtime_error("failed to allocate path list");
  }
  for (std::size_t i = 0; i < result.path.size(); ++i) {
    PyObject * item = Py_BuildValue(
      "[ddd]",
      result.path[i].x,
      result.path[i].y,
      result.path[i].z);
    if (item == nullptr) {
      Py_DECREF(path);
      throw std::runtime_error("failed to allocate path point");
    }
    PyList_SET_ITEM(path, static_cast<Py_ssize_t>(i), item);
  }
  return path;
}

PyObject * resultToPy(const octo::PlanResult & result)
{
  PyObject * out = PyDict_New();
  if (out == nullptr) {
    throw std::runtime_error("failed to allocate result dict");
  }
  setItem(out, "planner", PyUnicode_FromString("octoplanner3d"));
  setItem(out, "protocol_version", PyLong_FromLong(1));
  setItem(out, "ok", PyBool_FromLong(result.ok ? 1 : 0));
  if (!result.ok) {
    setItem(out, "error", PyUnicode_FromString("empty path"));
  }
  setItem(out, "path", pathFromResult(result));
  setItem(out, "reached_goal", PyBool_FromLong(result.reached_goal ? 1 : 0));
  setItem(out, "diagnostics", diagnosticsFromResult(result));
  return out;
}

PyObject * errorResult(const std::string & message)
{
  PyObject * out = PyDict_New();
  if (out == nullptr) {
    return nullptr;
  }
  try {
    setItem(out, "planner", PyUnicode_FromString("octoplanner3d"));
    setItem(out, "protocol_version", PyLong_FromLong(1));
    setItem(out, "ok", PyBool_FromLong(0));
    setItem(out, "error", PyUnicode_FromString(message.c_str()));
    setItem(out, "diagnostics", diagnosticsForError());
  } catch (...) {
    Py_DECREF(out);
    throw;
  }
  return out;
}

octo::PlanResult runPlanWithoutGil(const octo::PlanRequest & request)
{
  PyThreadState * state = PyEval_SaveThread();
  try {
    octo::PlanResult result = octo::runPlan(request);
    PyEval_RestoreThread(state);
    return result;
  } catch (...) {
    PyEval_RestoreThread(state);
    throw;
  }
}

PyObject * pyPlan(PyObject *, PyObject * args)
{
  PyObject * payload = nullptr;
  if (!PyArg_ParseTuple(args, "O!:plan", &PyDict_Type, &payload)) {
    return nullptr;
  }

  try {
    const octo::PlanRequest request = requestFromPayload(payload);
    return resultToPy(runPlanWithoutGil(request));
  } catch (const std::exception & exc) {
    try {
      return errorResult(exc.what());
    } catch (const std::exception & conversion_exc) {
      PyErr_SetString(PyExc_RuntimeError, conversion_exc.what());
      return nullptr;
    }
  }
}

PyMethodDef METHODS[] = {
  {"plan", pyPlan, METH_VARARGS, "Run OctoPlanner3D in-process from a planner payload."},
  {nullptr, nullptr, 0, nullptr},
};

PyModuleDef MODULE = {
  PyModuleDef_HEAD_INIT,
  "_native",
  "OctoPlanner3D runtime kernel for global planning (ROS2-free)",
  -1,
  METHODS,
};

}  // namespace

PyMODINIT_FUNC PyInit__native()
{
  return PyModule_Create(&MODULE);
}
