// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from unitree_go:msg/MotorCmds.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__MOTOR_CMDS__STRUCT_H_
#define UNITREE_GO__MSG__DETAIL__MOTOR_CMDS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'cmds'
#include "unitree_go/msg/detail/motor_cmd__struct.h"

// Struct defined in msg/MotorCmds in the package unitree_go.
typedef struct unitree_go__msg__MotorCmds
{
  unitree_go__msg__MotorCmd__Sequence cmds;
} unitree_go__msg__MotorCmds;

// Struct for a sequence of unitree_go__msg__MotorCmds.
typedef struct unitree_go__msg__MotorCmds__Sequence
{
  unitree_go__msg__MotorCmds * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} unitree_go__msg__MotorCmds__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UNITREE_GO__MSG__DETAIL__MOTOR_CMDS__STRUCT_H_
