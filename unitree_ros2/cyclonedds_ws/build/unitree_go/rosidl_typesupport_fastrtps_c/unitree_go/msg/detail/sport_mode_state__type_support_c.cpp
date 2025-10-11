// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from unitree_go:msg/SportModeState.idl
// generated code does not contain a copyright notice
#include "unitree_go/msg/detail/sport_mode_state__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "unitree_go/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "unitree_go/msg/detail/sport_mode_state__struct.h"
#include "unitree_go/msg/detail/sport_mode_state__functions.h"
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

#include "unitree_go/msg/detail/imu_state__functions.h"  // imu_state
#include "unitree_go/msg/detail/time_spec__functions.h"  // stamp

// forward declare type support functions
size_t get_serialized_size_unitree_go__msg__IMUState(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_unitree_go__msg__IMUState(
  bool & full_bounded,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, unitree_go, msg, IMUState)();
size_t get_serialized_size_unitree_go__msg__TimeSpec(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_unitree_go__msg__TimeSpec(
  bool & full_bounded,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, unitree_go, msg, TimeSpec)();


using _SportModeState__ros_msg_type = unitree_go__msg__SportModeState;

static bool _SportModeState__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _SportModeState__ros_msg_type * ros_message = static_cast<const _SportModeState__ros_msg_type *>(untyped_ros_message);
  // Field name: stamp
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, unitree_go, msg, TimeSpec
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->stamp, cdr))
    {
      return false;
    }
  }

  // Field name: error_code
  {
    cdr << ros_message->error_code;
  }

  // Field name: imu_state
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, unitree_go, msg, IMUState
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->imu_state, cdr))
    {
      return false;
    }
  }

  // Field name: mode
  {
    cdr << ros_message->mode;
  }

  // Field name: progress
  {
    cdr << ros_message->progress;
  }

  // Field name: gait_type
  {
    cdr << ros_message->gait_type;
  }

  // Field name: foot_raise_height
  {
    cdr << ros_message->foot_raise_height;
  }

  // Field name: position
  {
    size_t size = 3;
    auto array_ptr = ros_message->position;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: body_height
  {
    cdr << ros_message->body_height;
  }

  // Field name: velocity
  {
    size_t size = 3;
    auto array_ptr = ros_message->velocity;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: yaw_speed
  {
    cdr << ros_message->yaw_speed;
  }

  // Field name: range_obstacle
  {
    size_t size = 4;
    auto array_ptr = ros_message->range_obstacle;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: foot_force
  {
    size_t size = 4;
    auto array_ptr = ros_message->foot_force;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: foot_position_body
  {
    size_t size = 12;
    auto array_ptr = ros_message->foot_position_body;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: foot_speed_body
  {
    size_t size = 12;
    auto array_ptr = ros_message->foot_speed_body;
    cdr.serializeArray(array_ptr, size);
  }

  return true;
}

static bool _SportModeState__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _SportModeState__ros_msg_type * ros_message = static_cast<_SportModeState__ros_msg_type *>(untyped_ros_message);
  // Field name: stamp
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, unitree_go, msg, TimeSpec
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->stamp))
    {
      return false;
    }
  }

  // Field name: error_code
  {
    cdr >> ros_message->error_code;
  }

  // Field name: imu_state
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, unitree_go, msg, IMUState
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->imu_state))
    {
      return false;
    }
  }

  // Field name: mode
  {
    cdr >> ros_message->mode;
  }

  // Field name: progress
  {
    cdr >> ros_message->progress;
  }

  // Field name: gait_type
  {
    cdr >> ros_message->gait_type;
  }

  // Field name: foot_raise_height
  {
    cdr >> ros_message->foot_raise_height;
  }

  // Field name: position
  {
    size_t size = 3;
    auto array_ptr = ros_message->position;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: body_height
  {
    cdr >> ros_message->body_height;
  }

  // Field name: velocity
  {
    size_t size = 3;
    auto array_ptr = ros_message->velocity;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: yaw_speed
  {
    cdr >> ros_message->yaw_speed;
  }

  // Field name: range_obstacle
  {
    size_t size = 4;
    auto array_ptr = ros_message->range_obstacle;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: foot_force
  {
    size_t size = 4;
    auto array_ptr = ros_message->foot_force;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: foot_position_body
  {
    size_t size = 12;
    auto array_ptr = ros_message->foot_position_body;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: foot_speed_body
  {
    size_t size = 12;
    auto array_ptr = ros_message->foot_speed_body;
    cdr.deserializeArray(array_ptr, size);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_unitree_go
size_t get_serialized_size_unitree_go__msg__SportModeState(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _SportModeState__ros_msg_type * ros_message = static_cast<const _SportModeState__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name stamp

  current_alignment += get_serialized_size_unitree_go__msg__TimeSpec(
    &(ros_message->stamp), current_alignment);
  // field.name error_code
  {
    size_t item_size = sizeof(ros_message->error_code);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name imu_state

  current_alignment += get_serialized_size_unitree_go__msg__IMUState(
    &(ros_message->imu_state), current_alignment);
  // field.name mode
  {
    size_t item_size = sizeof(ros_message->mode);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name progress
  {
    size_t item_size = sizeof(ros_message->progress);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name gait_type
  {
    size_t item_size = sizeof(ros_message->gait_type);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name foot_raise_height
  {
    size_t item_size = sizeof(ros_message->foot_raise_height);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name position
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->position;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name body_height
  {
    size_t item_size = sizeof(ros_message->body_height);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name velocity
  {
    size_t array_size = 3;
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
  // field.name range_obstacle
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->range_obstacle;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name foot_force
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->foot_force;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name foot_position_body
  {
    size_t array_size = 12;
    auto array_ptr = ros_message->foot_position_body;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name foot_speed_body
  {
    size_t array_size = 12;
    auto array_ptr = ros_message->foot_speed_body;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _SportModeState__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_unitree_go__msg__SportModeState(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_unitree_go
size_t max_serialized_size_unitree_go__msg__SportModeState(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: stamp
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_unitree_go__msg__TimeSpec(
        full_bounded, current_alignment);
    }
  }
  // member: error_code
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: imu_state
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_unitree_go__msg__IMUState(
        full_bounded, current_alignment);
    }
  }
  // member: mode
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: progress
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: gait_type
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
  // member: position
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: body_height
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: velocity
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: yaw_speed
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: range_obstacle
  {
    size_t array_size = 4;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: foot_force
  {
    size_t array_size = 4;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: foot_position_body
  {
    size_t array_size = 12;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: foot_speed_body
  {
    size_t array_size = 12;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _SportModeState__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_unitree_go__msg__SportModeState(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_SportModeState = {
  "unitree_go::msg",
  "SportModeState",
  _SportModeState__cdr_serialize,
  _SportModeState__cdr_deserialize,
  _SportModeState__get_serialized_size,
  _SportModeState__max_serialized_size
};

static rosidl_message_type_support_t _SportModeState__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_SportModeState,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, unitree_go, msg, SportModeState)() {
  return &_SportModeState__type_support;
}

#if defined(__cplusplus)
}
#endif
