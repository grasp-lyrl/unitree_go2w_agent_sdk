// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from unitree_hg:msg/MainBoardState.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_HG__MSG__DETAIL__MAIN_BOARD_STATE__STRUCT_H_
#define UNITREE_HG__MSG__DETAIL__MAIN_BOARD_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/MainBoardState in the package unitree_hg.
typedef struct unitree_hg__msg__MainBoardState
{
  uint16_t fan_state[6];
  int16_t temperature[6];
  float value[6];
  uint32_t state[6];
} unitree_hg__msg__MainBoardState;

// Struct for a sequence of unitree_hg__msg__MainBoardState.
typedef struct unitree_hg__msg__MainBoardState__Sequence
{
  unitree_hg__msg__MainBoardState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} unitree_hg__msg__MainBoardState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UNITREE_HG__MSG__DETAIL__MAIN_BOARD_STATE__STRUCT_H_
