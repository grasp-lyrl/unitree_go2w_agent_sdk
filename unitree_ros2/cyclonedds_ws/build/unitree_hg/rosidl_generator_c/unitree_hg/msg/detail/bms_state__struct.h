// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from unitree_hg:msg/BmsState.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_HG__MSG__DETAIL__BMS_STATE__STRUCT_H_
#define UNITREE_HG__MSG__DETAIL__BMS_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/BmsState in the package unitree_hg.
typedef struct unitree_hg__msg__BmsState
{
  uint8_t version_high;
  uint8_t version_low;
  uint8_t fn;
  uint16_t cell_vol[40];
  uint32_t bmsvoltage[3];
  int32_t current;
  uint8_t soc;
  uint8_t soh;
  int16_t temperature[12];
  uint16_t cycle;
  uint16_t manufacturer_date;
  uint32_t bmsstate[5];
  uint32_t reserve[3];
} unitree_hg__msg__BmsState;

// Struct for a sequence of unitree_hg__msg__BmsState.
typedef struct unitree_hg__msg__BmsState__Sequence
{
  unitree_hg__msg__BmsState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} unitree_hg__msg__BmsState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UNITREE_HG__MSG__DETAIL__BMS_STATE__STRUCT_H_
