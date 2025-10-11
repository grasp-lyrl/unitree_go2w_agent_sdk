// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from unitree_hg:msg/HandState.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "unitree_hg/msg/detail/hand_state__struct.h"
#include "unitree_hg/msg/detail/hand_state__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

// Nested array functions includes
#include "unitree_hg/msg/detail/motor_state__functions.h"
#include "unitree_hg/msg/detail/press_sensor_state__functions.h"
// end nested array functions include
bool unitree_hg__msg__motor_state__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * unitree_hg__msg__motor_state__convert_to_py(void * raw_ros_message);
bool unitree_hg__msg__imu_state__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * unitree_hg__msg__imu_state__convert_to_py(void * raw_ros_message);
bool unitree_hg__msg__press_sensor_state__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * unitree_hg__msg__press_sensor_state__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool unitree_hg__msg__hand_state__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[37];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("unitree_hg.msg._hand_state.HandState", full_classname_dest, 36) == 0);
  }
  unitree_hg__msg__HandState * ros_message = _ros_message;
  {  // motor_state
    PyObject * field = PyObject_GetAttrString(_pymsg, "motor_state");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'motor_state'");
    if (!seq_field) {
      Py_DECREF(field);
      return false;
    }
    Py_ssize_t size = PySequence_Size(field);
    if (-1 == size) {
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    if (!unitree_hg__msg__MotorState__Sequence__init(&(ros_message->motor_state), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create unitree_hg__msg__MotorState__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    unitree_hg__msg__MotorState * dest = ros_message->motor_state.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!unitree_hg__msg__motor_state__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }
  {  // imu_state
    PyObject * field = PyObject_GetAttrString(_pymsg, "imu_state");
    if (!field) {
      return false;
    }
    if (!unitree_hg__msg__imu_state__convert_from_py(field, &ros_message->imu_state)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // press_sensor_state
    PyObject * field = PyObject_GetAttrString(_pymsg, "press_sensor_state");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'press_sensor_state'");
    if (!seq_field) {
      Py_DECREF(field);
      return false;
    }
    Py_ssize_t size = PySequence_Size(field);
    if (-1 == size) {
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    if (!unitree_hg__msg__PressSensorState__Sequence__init(&(ros_message->press_sensor_state), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create unitree_hg__msg__PressSensorState__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    unitree_hg__msg__PressSensorState * dest = ros_message->press_sensor_state.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!unitree_hg__msg__press_sensor_state__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }
  {  // power_v
    PyObject * field = PyObject_GetAttrString(_pymsg, "power_v");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->power_v = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // power_a
    PyObject * field = PyObject_GetAttrString(_pymsg, "power_a");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->power_a = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // reserve
    PyObject * field = PyObject_GetAttrString(_pymsg, "reserve");
    if (!field) {
      return false;
    }
    {
      // TODO(dirk-thomas) use a better way to check the type before casting
      assert(field->ob_type != NULL);
      assert(field->ob_type->tp_name != NULL);
      assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
      PyArrayObject * seq_field = (PyArrayObject *)field;
      Py_INCREF(seq_field);
      assert(PyArray_NDIM(seq_field) == 1);
      assert(PyArray_TYPE(seq_field) == NPY_UINT32);
      Py_ssize_t size = 2;
      uint32_t * dest = ros_message->reserve;
      for (Py_ssize_t i = 0; i < size; ++i) {
        uint32_t tmp = *(npy_uint32 *)PyArray_GETPTR1(seq_field, i);
        memcpy(&dest[i], &tmp, sizeof(uint32_t));
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * unitree_hg__msg__hand_state__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of HandState */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("unitree_hg.msg._hand_state");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "HandState");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  unitree_hg__msg__HandState * ros_message = (unitree_hg__msg__HandState *)raw_ros_message;
  {  // motor_state
    PyObject * field = NULL;
    size_t size = ros_message->motor_state.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    unitree_hg__msg__MotorState * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->motor_state.data[i]);
      PyObject * pyitem = unitree_hg__msg__motor_state__convert_to_py(item);
      if (!pyitem) {
        Py_DECREF(field);
        return NULL;
      }
      int rc = PyList_SetItem(field, i, pyitem);
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "motor_state", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // imu_state
    PyObject * field = NULL;
    field = unitree_hg__msg__imu_state__convert_to_py(&ros_message->imu_state);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "imu_state", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // press_sensor_state
    PyObject * field = NULL;
    size_t size = ros_message->press_sensor_state.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    unitree_hg__msg__PressSensorState * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->press_sensor_state.data[i]);
      PyObject * pyitem = unitree_hg__msg__press_sensor_state__convert_to_py(item);
      if (!pyitem) {
        Py_DECREF(field);
        return NULL;
      }
      int rc = PyList_SetItem(field, i, pyitem);
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "press_sensor_state", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // power_v
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->power_v);
    {
      int rc = PyObject_SetAttrString(_pymessage, "power_v", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // power_a
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->power_a);
    {
      int rc = PyObject_SetAttrString(_pymessage, "power_a", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // reserve
    PyObject * field = NULL;
    field = PyObject_GetAttrString(_pymessage, "reserve");
    if (!field) {
      return NULL;
    }
    assert(field->ob_type != NULL);
    assert(field->ob_type->tp_name != NULL);
    assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
    PyArrayObject * seq_field = (PyArrayObject *)field;
    assert(PyArray_NDIM(seq_field) == 1);
    assert(PyArray_TYPE(seq_field) == NPY_UINT32);
    assert(sizeof(npy_uint32) == sizeof(uint32_t));
    npy_uint32 * dst = (npy_uint32 *)PyArray_GETPTR1(seq_field, 0);
    uint32_t * src = &(ros_message->reserve[0]);
    memcpy(dst, src, 2 * sizeof(uint32_t));
    Py_DECREF(field);
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
