// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from unitree_go:msg/WirelessController.idl
// generated code does not contain a copyright notice
#include "unitree_go/msg/detail/wireless_controller__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "unitree_go/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "unitree_go/msg/detail/wireless_controller__struct.h"
#include "unitree_go/msg/detail/wireless_controller__functions.h"
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


using _WirelessController__ros_msg_type = unitree_go__msg__WirelessController;

static bool _WirelessController__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _WirelessController__ros_msg_type * ros_message = static_cast<const _WirelessController__ros_msg_type *>(untyped_ros_message);
  // Field name: lx
  {
    cdr << ros_message->lx;
  }

  // Field name: ly
  {
    cdr << ros_message->ly;
  }

  // Field name: rx
  {
    cdr << ros_message->rx;
  }

  // Field name: ry
  {
    cdr << ros_message->ry;
  }

  // Field name: keys
  {
    cdr << ros_message->keys;
  }

  return true;
}

static bool _WirelessController__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _WirelessController__ros_msg_type * ros_message = static_cast<_WirelessController__ros_msg_type *>(untyped_ros_message);
  // Field name: lx
  {
    cdr >> ros_message->lx;
  }

  // Field name: ly
  {
    cdr >> ros_message->ly;
  }

  // Field name: rx
  {
    cdr >> ros_message->rx;
  }

  // Field name: ry
  {
    cdr >> ros_message->ry;
  }

  // Field name: keys
  {
    cdr >> ros_message->keys;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_unitree_go
size_t get_serialized_size_unitree_go__msg__WirelessController(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _WirelessController__ros_msg_type * ros_message = static_cast<const _WirelessController__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name lx
  {
    size_t item_size = sizeof(ros_message->lx);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ly
  {
    size_t item_size = sizeof(ros_message->ly);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name rx
  {
    size_t item_size = sizeof(ros_message->rx);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ry
  {
    size_t item_size = sizeof(ros_message->ry);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name keys
  {
    size_t item_size = sizeof(ros_message->keys);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _WirelessController__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_unitree_go__msg__WirelessController(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_unitree_go
size_t max_serialized_size_unitree_go__msg__WirelessController(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: lx
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: ly
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: rx
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: ry
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: keys
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _WirelessController__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_unitree_go__msg__WirelessController(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_WirelessController = {
  "unitree_go::msg",
  "WirelessController",
  _WirelessController__cdr_serialize,
  _WirelessController__cdr_deserialize,
  _WirelessController__get_serialized_size,
  _WirelessController__max_serialized_size
};

static rosidl_message_type_support_t _WirelessController__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_WirelessController,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, unitree_go, msg, WirelessController)() {
  return &_WirelessController__type_support;
}

#if defined(__cplusplus)
}
#endif
