// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from unitree_hg:msg/HandCmd.idl
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
#include "unitree_hg/msg/detail/hand_cmd__struct.h"
#include "unitree_hg/msg/detail/hand_cmd__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

// Nested array functions includes
#include "unitree_hg/msg/detail/motor_cmd__functions.h"
// end nested array functions include
bool unitree_hg__msg__motor_cmd__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * unitree_hg__msg__motor_cmd__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool unitree_hg__msg__hand_cmd__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[33];
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
    assert(strncmp("unitree_hg.msg._hand_cmd.HandCmd", full_classname_dest, 32) == 0);
  }
  unitree_hg__msg__HandCmd * ros_message = _ros_message;
  {  // motor_cmd
    PyObject * field = PyObject_GetAttrString(_pymsg, "motor_cmd");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'motor_cmd'");
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
    if (!unitree_hg__msg__MotorCmd__Sequence__init(&(ros_message->motor_cmd), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create unitree_hg__msg__MotorCmd__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    unitree_hg__msg__MotorCmd * dest = ros_message->motor_cmd.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!unitree_hg__msg__motor_cmd__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * unitree_hg__msg__hand_cmd__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of HandCmd */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("unitree_hg.msg._hand_cmd");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "HandCmd");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  unitree_hg__msg__HandCmd * ros_message = (unitree_hg__msg__HandCmd *)raw_ros_message;
  {  // motor_cmd
    PyObject * field = NULL;
    size_t size = ros_message->motor_cmd.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    unitree_hg__msg__MotorCmd * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->motor_cmd.data[i]);
      PyObject * pyitem = unitree_hg__msg__motor_cmd__convert_to_py(item);
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
      int rc = PyObject_SetAttrString(_pymessage, "motor_cmd", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
