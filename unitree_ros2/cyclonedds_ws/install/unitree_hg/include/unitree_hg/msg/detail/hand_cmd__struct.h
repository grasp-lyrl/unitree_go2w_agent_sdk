// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from unitree_hg:msg/HandCmd.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_HG__MSG__DETAIL__HAND_CMD__STRUCT_H_
#define UNITREE_HG__MSG__DETAIL__HAND_CMD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'motor_cmd'
#include "unitree_hg/msg/detail/motor_cmd__struct.h"

// Struct defined in msg/HandCmd in the package unitree_hg.
typedef struct unitree_hg__msg__HandCmd
{
  unitree_hg__msg__MotorCmd__Sequence motor_cmd;
} unitree_hg__msg__HandCmd;

// Struct for a sequence of unitree_hg__msg__HandCmd.
typedef struct unitree_hg__msg__HandCmd__Sequence
{
  unitree_hg__msg__HandCmd * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} unitree_hg__msg__HandCmd__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UNITREE_HG__MSG__DETAIL__HAND_CMD__STRUCT_H_
