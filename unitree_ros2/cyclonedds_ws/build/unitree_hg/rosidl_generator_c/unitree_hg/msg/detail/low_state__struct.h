// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from unitree_hg:msg/LowState.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_HG__MSG__DETAIL__LOW_STATE__STRUCT_H_
#define UNITREE_HG__MSG__DETAIL__LOW_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'imu_state'
#include "unitree_hg/msg/detail/imu_state__struct.h"
// Member 'motor_state'
#include "unitree_hg/msg/detail/motor_state__struct.h"

// Struct defined in msg/LowState in the package unitree_hg.
typedef struct unitree_hg__msg__LowState
{
  uint32_t version[2];
  uint8_t mode_pr;
  uint8_t mode_machine;
  uint32_t tick;
  unitree_hg__msg__IMUState imu_state;
  unitree_hg__msg__MotorState motor_state[35];
  uint8_t wireless_remote[40];
  uint32_t reserve[4];
  uint32_t crc;
} unitree_hg__msg__LowState;

// Struct for a sequence of unitree_hg__msg__LowState.
typedef struct unitree_hg__msg__LowState__Sequence
{
  unitree_hg__msg__LowState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} unitree_hg__msg__LowState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UNITREE_HG__MSG__DETAIL__LOW_STATE__STRUCT_H_
