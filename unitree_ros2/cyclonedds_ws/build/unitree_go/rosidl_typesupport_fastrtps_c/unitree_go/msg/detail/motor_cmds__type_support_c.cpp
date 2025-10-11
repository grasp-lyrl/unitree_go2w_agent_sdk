// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from unitree_go:msg/MotorCmds.idl
// generated code does not contain a copyright notice
#include "unitree_go/msg/detail/motor_cmds__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "unitree_go/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "unitree_go/msg/detail/motor_cmds__struct.h"
#include "unitree_go/msg/detail/motor_cmds__functions.h"
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

#include "unitree_go/msg/detail/motor_cmd__functions.h"  // cmds

// forward declare type support functions
size_t get_serialized_size_unitree_go__msg__MotorCmd(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_unitree_go__msg__MotorCmd(
  bool & full_bounded,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, unitree_go, msg, MotorCmd)();


using _MotorCmds__ros_msg_type = unitree_go__msg__MotorCmds;

static bool _MotorCmds__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _MotorCmds__ros_msg_type * ros_message = static_cast<const _MotorCmds__ros_msg_type *>(untyped_ros_message);
  // Field name: cmds
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, unitree_go, msg, MotorCmd
      )()->data);
    size_t size = ros_message->cmds.size;
    auto array_ptr = ros_message->cmds.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_serialize(
          &array_ptr[i], cdr))
      {
        return false;
      }
    }
  }

  return true;
}

static bool _MotorCmds__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _MotorCmds__ros_msg_type * ros_message = static_cast<_MotorCmds__ros_msg_type *>(untyped_ros_message);
  // Field name: cmds
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, unitree_go, msg, MotorCmd
      )()->data);
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->cmds.data) {
      unitree_go__msg__MotorCmd__Sequence__fini(&ros_message->cmds);
    }
    if (!unitree_go__msg__MotorCmd__Sequence__init(&ros_message->cmds, size)) {
      return "failed to create array for field 'cmds'";
    }
    auto array_ptr = ros_message->cmds.data;
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_deserialize(
          cdr, &array_ptr[i]))
      {
        return false;
      }
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_unitree_go
size_t get_serialized_size_unitree_go__msg__MotorCmds(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _MotorCmds__ros_msg_type * ros_message = static_cast<const _MotorCmds__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name cmds
  {
    size_t array_size = ros_message->cmds.size;
    auto array_ptr = ros_message->cmds.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_unitree_go__msg__MotorCmd(
        &array_ptr[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static uint32_t _MotorCmds__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_unitree_go__msg__MotorCmds(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_unitree_go
size_t max_serialized_size_unitree_go__msg__MotorCmds(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: cmds
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_unitree_go__msg__MotorCmd(
        full_bounded, current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static size_t _MotorCmds__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_unitree_go__msg__MotorCmds(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_MotorCmds = {
  "unitree_go::msg",
  "MotorCmds",
  _MotorCmds__cdr_serialize,
  _MotorCmds__cdr_deserialize,
  _MotorCmds__get_serialized_size,
  _MotorCmds__max_serialized_size
};

static rosidl_message_type_support_t _MotorCmds__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_MotorCmds,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, unitree_go, msg, MotorCmds)() {
  return &_MotorCmds__type_support;
}

#if defined(__cplusplus)
}
#endif
