// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from unitree_hg:msg/MotorCmd.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_HG__MSG__DETAIL__MOTOR_CMD__STRUCT_H_
#define UNITREE_HG__MSG__DETAIL__MOTOR_CMD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/MotorCmd in the package unitree_hg.
typedef struct unitree_hg__msg__MotorCmd
{
  uint8_t mode;
  float q;
  float dq;
  float tau;
  float kp;
  float kd;
  uint32_t reserve;
} unitree_hg__msg__MotorCmd;

// Struct for a sequence of unitree_hg__msg__MotorCmd.
typedef struct unitree_hg__msg__MotorCmd__Sequence
{
  unitree_hg__msg__MotorCmd * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} unitree_hg__msg__MotorCmd__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UNITREE_HG__MSG__DETAIL__MOTOR_CMD__STRUCT_H_
