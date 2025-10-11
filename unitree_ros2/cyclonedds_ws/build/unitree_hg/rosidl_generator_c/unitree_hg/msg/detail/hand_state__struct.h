// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from unitree_hg:msg/HandState.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_HG__MSG__DETAIL__HAND_STATE__STRUCT_H_
#define UNITREE_HG__MSG__DETAIL__HAND_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'motor_state'
#include "unitree_hg/msg/detail/motor_state__struct.h"
// Member 'imu_state'
#include "unitree_hg/msg/detail/imu_state__struct.h"
// Member 'press_sensor_state'
#include "unitree_hg/msg/detail/press_sensor_state__struct.h"

// Struct defined in msg/HandState in the package unitree_hg.
typedef struct unitree_hg__msg__HandState
{
  unitree_hg__msg__MotorState__Sequence motor_state;
  unitree_hg__msg__IMUState imu_state;
  unitree_hg__msg__PressSensorState__Sequence press_sensor_state;
  float power_v;
  float power_a;
  uint32_t reserve[2];
} unitree_hg__msg__HandState;

// Struct for a sequence of unitree_hg__msg__HandState.
typedef struct unitree_hg__msg__HandState__Sequence
{
  unitree_hg__msg__HandState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} unitree_hg__msg__HandState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UNITREE_HG__MSG__DETAIL__HAND_STATE__STRUCT_H_
