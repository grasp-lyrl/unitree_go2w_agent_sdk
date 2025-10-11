// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from unitree_hg:msg/MainBoardState.idl
// generated code does not contain a copyright notice
#include "unitree_hg/msg/detail/main_board_state__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "unitree_hg/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "unitree_hg/msg/detail/main_board_state__struct.h"
#include "unitree_hg/msg/detail/main_board_state__functions.h"
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


using _MainBoardState__ros_msg_type = unitree_hg__msg__MainBoardState;

static bool _MainBoardState__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _MainBoardState__ros_msg_type * ros_message = static_cast<const _MainBoardState__ros_msg_type *>(untyped_ros_message);
  // Field name: fan_state
  {
    size_t size = 6;
    auto array_ptr = ros_message->fan_state;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: temperature
  {
    size_t size = 6;
    auto array_ptr = ros_message->temperature;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: value
  {
    size_t size = 6;
    auto array_ptr = ros_message->value;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: state
  {
    size_t size = 6;
    auto array_ptr = ros_message->state;
    cdr.serializeArray(array_ptr, size);
  }

  return true;
}

static bool _MainBoardState__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _MainBoardState__ros_msg_type * ros_message = static_cast<_MainBoardState__ros_msg_type *>(untyped_ros_message);
  // Field name: fan_state
  {
    size_t size = 6;
    auto array_ptr = ros_message->fan_state;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: temperature
  {
    size_t size = 6;
    auto array_ptr = ros_message->temperature;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: value
  {
    size_t size = 6;
    auto array_ptr = ros_message->value;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: state
  {
    size_t size = 6;
    auto array_ptr = ros_message->state;
    cdr.deserializeArray(array_ptr, size);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_unitree_hg
size_t get_serialized_size_unitree_hg__msg__MainBoardState(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _MainBoardState__ros_msg_type * ros_message = static_cast<const _MainBoardState__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name fan_state
  {
    size_t array_size = 6;
    auto array_ptr = ros_message->fan_state;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name temperature
  {
    size_t array_size = 6;
    auto array_ptr = ros_message->temperature;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name value
  {
    size_t array_size = 6;
    auto array_ptr = ros_message->value;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name state
  {
    size_t array_size = 6;
    auto array_ptr = ros_message->state;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _MainBoardState__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_unitree_hg__msg__MainBoardState(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_unitree_hg
size_t max_serialized_size_unitree_hg__msg__MainBoardState(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: fan_state
  {
    size_t array_size = 6;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: temperature
  {
    size_t array_size = 6;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: value
  {
    size_t array_size = 6;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: state
  {
    size_t array_size = 6;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _MainBoardState__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_unitree_hg__msg__MainBoardState(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_MainBoardState = {
  "unitree_hg::msg",
  "MainBoardState",
  _MainBoardState__cdr_serialize,
  _MainBoardState__cdr_deserialize,
  _MainBoardState__get_serialized_size,
  _MainBoardState__max_serialized_size
};

static rosidl_message_type_support_t _MainBoardState__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_MainBoardState,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, unitree_hg, msg, MainBoardState)() {
  return &_MainBoardState__type_support;
}

#if defined(__cplusplus)
}
#endif
