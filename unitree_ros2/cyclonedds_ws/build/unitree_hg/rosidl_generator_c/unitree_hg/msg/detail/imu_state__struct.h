// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from unitree_hg:msg/IMUState.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_HG__MSG__DETAIL__IMU_STATE__STRUCT_H_
#define UNITREE_HG__MSG__DETAIL__IMU_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/IMUState in the package unitree_hg.
typedef struct unitree_hg__msg__IMUState
{
  float quaternion[4];
  float gyroscope[3];
  float accelerometer[3];
  float rpy[3];
  int16_t temperature;
} unitree_hg__msg__IMUState;

// Struct for a sequence of unitree_hg__msg__IMUState.
typedef struct unitree_hg__msg__IMUState__Sequence
{
  unitree_hg__msg__IMUState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} unitree_hg__msg__IMUState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UNITREE_HG__MSG__DETAIL__IMU_STATE__STRUCT_H_
