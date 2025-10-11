// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from unitree_hg:msg/PressSensorState.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_HG__MSG__DETAIL__PRESS_SENSOR_STATE__STRUCT_H_
#define UNITREE_HG__MSG__DETAIL__PRESS_SENSOR_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/PressSensorState in the package unitree_hg.
typedef struct unitree_hg__msg__PressSensorState
{
  float pressure[12];
  float temperature[12];
} unitree_hg__msg__PressSensorState;

// Struct for a sequence of unitree_hg__msg__PressSensorState.
typedef struct unitree_hg__msg__PressSensorState__Sequence
{
  unitree_hg__msg__PressSensorState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} unitree_hg__msg__PressSensorState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UNITREE_HG__MSG__DETAIL__PRESS_SENSOR_STATE__STRUCT_H_
