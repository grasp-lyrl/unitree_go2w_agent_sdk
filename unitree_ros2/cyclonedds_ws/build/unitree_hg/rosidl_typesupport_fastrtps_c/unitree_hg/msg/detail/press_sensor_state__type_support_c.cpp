// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from unitree_hg:msg/PressSensorState.idl
// generated code does not contain a copyright notice
#include "unitree_hg/msg/detail/press_sensor_state__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "unitree_hg/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "unitree_hg/msg/detail/press_sensor_state__struct.h"
#include "unitree_hg/msg/detail/press_sensor_state__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _PressSensorState__ros_msg_type = unitree_hg__msg__PressSensorState;

static bool _PressSensorState__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _PressSensorState__ros_msg_type * ros_message = static_cast<const _PressSensorState__ros_msg_type *>(untyped_ros_message);
  // Field name: pressure
  {
    size_t size = 12;
    auto array_ptr = ros_message->pressure;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: temperature
  {
    size_t size = 12;
    auto array_ptr = ros_message->temperature;
    cdr.serializeArray(array_ptr, size);
  }

  return true;
}

static bool _PressSensorState__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _PressSensorState__ros_msg_type * ros_message = static_cast<_PressSensorState__ros_msg_type *>(untyped_ros_message);
  // Field name: pressure
  {
    size_t size = 12;
    auto array_ptr = ros_message->pressure;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: temperature
  {
    size_t size = 12;
    auto array_ptr = ros_message->temperature;
    cdr.deserializeArray(array_ptr, size);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_unitree_hg
size_t get_serialized_size_unitree_hg__msg__PressSensorState(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _PressSensorState__ros_msg_type * ros_message = static_cast<const _PressSensorState__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name pressure
  {
    size_t array_size = 12;
    auto array_ptr = ros_message->pressure;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name temperature
  {
    size_t array_size = 12;
    auto array_ptr = ros_message->temperature;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _PressSensorState__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_unitree_hg__msg__PressSensorState(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_unitree_hg
size_t max_serialized_size_unitree_hg__msg__PressSensorState(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: pressure
  {
    size_t array_size = 12;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: temperature
  {
    size_t array_size = 12;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _PressSensorState__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_unitree_hg__msg__PressSensorState(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_PressSensorState = {
  "unitree_hg::msg",
  "PressSensorState",
  _PressSensorState__cdr_serialize,
  _PressSensorState__cdr_deserialize,
  _PressSensorState__get_serialized_size,
  _PressSensorState__max_serialized_size
};

static rosidl_message_type_support_t _PressSensorState__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_PressSensorState,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, unitree_hg, msg, PressSensorState)() {
  return &_PressSensorState__type_support;
}

#if defined(__cplusplus)
}
#endif
