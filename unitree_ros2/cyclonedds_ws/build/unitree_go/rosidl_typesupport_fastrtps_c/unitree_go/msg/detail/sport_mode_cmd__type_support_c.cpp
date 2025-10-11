// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from unitree_go:msg/SportModeCmd.idl
// generated code does not contain a copyright notice
#include "unitree_go/msg/detail/sport_mode_cmd__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "unitree_go/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "unitree_go/msg/detail/sport_mode_cmd__struct.h"
#include "unitree_go/msg/detail/sport_mode_cmd__functions.h"
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

#include "unitree_go/msg/detail/bms_cmd__functions.h"  // bms_cmd
#include "unitree_go/msg/detail/path_point__functions.h"  // path_point

// forward declare type support functions
size_t get_serialized_size_unitree_go__msg__BmsCmd(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_unitree_go__msg__BmsCmd(
  bool & full_bounded,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, unitree_go, msg, BmsCmd)();
size_t get_serialized_size_unitree_go__msg__PathPoint(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_unitree_go__msg__PathPoint(
  bool & full_bounded,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, unitree_go, msg, PathPoint)();


using _SportModeCmd__ros_msg_type = unitree_go__msg__SportModeCmd;

static bool _SportModeCmd__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _SportModeCmd__ros_msg_type * ros_message = static_cast<const _SportModeCmd__ros_msg_type *>(untyped_ros_message);
  // Field name: mode
  {
    cdr << ros_message->mode;
  }

  // Field name: gait_type
  {
    cdr << ros_message->gait_type;
  }

  // Field name: speed_level
  {
    cdr << ros_message->speed_level;
  }

  // Field name: foot_raise_height
  {
    cdr << ros_message->foot_raise_height;
  }

  // Field name: body_height
  {
    cdr << ros_message->body_height;
  }

  // Field name: position
  {
    size_t size = 2;
    auto array_ptr = ros_message->position;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: euler
  {
    size_t size = 3;
    auto array_ptr = ros_message->euler;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: velocity
  {
    size_t size = 2;
    auto array_ptr = ros_message->velocity;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: yaw_speed
  {
    cdr << ros_message->yaw_speed;
  }

  // Field name: bms_cmd
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, unitree_go, msg, BmsCmd
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->bms_cmd, cdr))
    {
      return false;
    }
  }

  // Field name: path_point
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, unitree_go, msg, PathPoint
      )()->data);
    size_t size = 30;
    auto array_ptr = ros_message->path_point;
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

static bool _SportModeCmd__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _SportModeCmd__ros_msg_type * ros_message = static_cast<_SportModeCmd__ros_msg_type *>(untyped_ros_message);
  // Field name: mode
  {
    cdr >> ros_message->mode;
  }

  // Field name: gait_type
  {
    cdr >> ros_message->gait_type;
  }

  // Field name: speed_level
  {
    cdr >> ros_message->speed_level;
  }

  // Field name: foot_raise_height
  {
    cdr >> ros_message->foot_raise_height;
  }

  // Field name: body_height
  {
    cdr >> ros_message->body_height;
  }

  // Field name: position
  {
    size_t size = 2;
    auto array_ptr = ros_message->position;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: euler
  {
    size_t size = 3;
    auto array_ptr = ros_message->euler;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: velocity
  {
    size_t size = 2;
    auto array_ptr = ros_message->velocity;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: yaw_speed
  {
    cdr >> ros_message->yaw_speed;
  }

  // Field name: bms_cmd
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, unitree_go, msg, BmsCmd
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->bms_cmd))
    {
      return false;
    }
  }

  // Field name: path_point
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, unitree_go, msg, PathPoint
      )()->data);
    size_t size = 30;
    auto array_ptr = ros_message->path_point;
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
size_t get_serialized_size_unitree_go__msg__SportModeCmd(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _SportModeCmd__ros_msg_type * ros_message = static_cast<const _SportModeCmd__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name mode
  {
    size_t item_size = sizeof(ros_message->mode);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name gait_type
  {
    size_t item_size = sizeof(ros_message->gait_type);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name speed_level
  {
    size_t item_size = sizeof(ros_message->speed_level);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name foot_raise_height
  {
    size_t item_size = sizeof(ros_message->foot_raise_height);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name body_height
  {
    size_t item_size = sizeof(ros_message->body_height);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name position
  {
    size_t array_size = 2;
    auto array_ptr = ros_message->position;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name euler
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->euler;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name velocity
  {
    size_t array_size = 2;
    auto array_ptr = ros_message->velocity;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name yaw_speed
  {
    size_t item_size = sizeof(ros_message->yaw_speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name bms_cmd

  current_alignment += get_serialized_size_unitree_go__msg__BmsCmd(
    &(ros_message->bms_cmd), current_alignment);
  // field.name path_point
  {
    size_t array_size = 30;
    auto array_ptr = ros_message->path_point;

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_unitree_go__msg__PathPoint(
        &array_ptr[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static uint32_t _SportModeCmd__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_unitree_go__msg__SportModeCmd(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_unitree_go
size_t max_serialized_size_unitree_go__msg__SportModeCmd(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: mode
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: gait_type
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: speed_level
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: foot_raise_height
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: body_height
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: position
  {
    size_t array_size = 2;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: euler
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: velocity
  {
    size_t array_size = 2;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: yaw_speed
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: bms_cmd
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_unitree_go__msg__BmsCmd(
        full_bounded, current_alignment);
    }
  }
  // member: path_point
  {
    size_t array_size = 30;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_unitree_go__msg__PathPoint(
        full_bounded, current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static size_t _SportModeCmd__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_unitree_go__msg__SportModeCmd(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_SportModeCmd = {
  "unitree_go::msg",
  "SportModeCmd",
  _SportModeCmd__cdr_serialize,
  _SportModeCmd__cdr_deserialize,
  _SportModeCmd__get_serialized_size,
  _SportModeCmd__max_serialized_size
};

static rosidl_message_type_support_t _SportModeCmd__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_SportModeCmd,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, unitree_go, msg, SportModeCmd)() {
  return &_SportModeCmd__type_support;
}

#if defined(__cplusplus)
}
#endif
