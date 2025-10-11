// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from unitree_hg:msg/LowState.idl
// generated code does not contain a copyright notice
#include "unitree_hg/msg/detail/low_state__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "unitree_hg/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "unitree_hg/msg/detail/low_state__struct.h"
#include "unitree_hg/msg/detail/low_state__functions.h"
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

#include "unitree_hg/msg/detail/imu_state__functions.h"  // imu_state
#include "unitree_hg/msg/detail/motor_state__functions.h"  // motor_state

// forward declare type support functions
size_t get_serialized_size_unitree_hg__msg__IMUState(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_unitree_hg__msg__IMUState(
  bool & full_bounded,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, unitree_hg, msg, IMUState)();
size_t get_serialized_size_unitree_hg__msg__MotorState(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_unitree_hg__msg__MotorState(
  bool & full_bounded,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, unitree_hg, msg, MotorState)();


using _LowState__ros_msg_type = unitree_hg__msg__LowState;

static bool _LowState__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _LowState__ros_msg_type * ros_message = static_cast<const _LowState__ros_msg_type *>(untyped_ros_message);
  // Field name: version
  {
    size_t size = 2;
    auto array_ptr = ros_message->version;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: mode_pr
  {
    cdr << ros_message->mode_pr;
  }

  // Field name: mode_machine
  {
    cdr << ros_message->mode_machine;
  }

  // Field name: tick
  {
    cdr << ros_message->tick;
  }

  // Field name: imu_state
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, unitree_hg, msg, IMUState
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->imu_state, cdr))
    {
      return false;
    }
  }

  // Field name: motor_state
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, unitree_hg, msg, MotorState
      )()->data);
    size_t size = 35;
    auto array_ptr = ros_message->motor_state;
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_serialize(
          &array_ptr[i], cdr))
      {
        return false;
      }
    }
  }

  // Field name: wireless_remote
  {
    size_t size = 40;
    auto array_ptr = ros_message->wireless_remote;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: reserve
  {
    size_t size = 4;
    auto array_ptr = ros_message->reserve;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: crc
  {
    cdr << ros_message->crc;
  }

  return true;
}

static bool _LowState__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _LowState__ros_msg_type * ros_message = static_cast<_LowState__ros_msg_type *>(untyped_ros_message);
  // Field name: version
  {
    size_t size = 2;
    auto array_ptr = ros_message->version;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: mode_pr
  {
    cdr >> ros_message->mode_pr;
  }

  // Field name: mode_machine
  {
    cdr >> ros_message->mode_machine;
  }

  // Field name: tick
  {
    cdr >> ros_message->tick;
  }

  // Field name: imu_state
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, unitree_hg, msg, IMUState
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->imu_state))
    {
      return false;
    }
  }

  // Field name: motor_state
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, unitree_hg, msg, MotorState
      )()->data);
    size_t size = 35;
    auto array_ptr = ros_message->motor_state;
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_deserialize(
          cdr, &array_ptr[i]))
      {
        return false;
      }
    }
  }

  // Field name: wireless_remote
  {
    size_t size = 40;
    auto array_ptr = ros_message->wireless_remote;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: reserve
  {
    size_t size = 4;
    auto array_ptr = ros_message->reserve;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: crc
  {
    cdr >> ros_message->crc;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_unitree_hg
size_t get_serialized_size_unitree_hg__msg__LowState(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _LowState__ros_msg_type * ros_message = static_cast<const _LowState__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name version
  {
    size_t array_size = 2;
    auto array_ptr = ros_message->version;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name mode_pr
  {
    size_t item_size = sizeof(ros_message->mode_pr);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name mode_machine
  {
    size_t item_size = sizeof(ros_message->mode_machine);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name tick
  {
    size_t item_size = sizeof(ros_message->tick);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name imu_state

  current_alignment += get_serialized_size_unitree_hg__msg__IMUState(
    &(ros_message->imu_state), current_alignment);
  // field.name motor_state
  {
    size_t array_size = 35;
    auto array_ptr = ros_message->motor_state;

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_unitree_hg__msg__MotorState(
        &array_ptr[index], current_alignment);
    }
  }
  // field.name wireless_remote
  {
    size_t array_size = 40;
    auto array_ptr = ros_message->wireless_remote;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name reserve
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->reserve;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name crc
  {
    size_t item_size = sizeof(ros_message->crc);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _LowState__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_unitree_hg__msg__LowState(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_unitree_hg
size_t max_serialized_size_unitree_hg__msg__LowState(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: version
  {
    size_t array_size = 2;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: mode_pr
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: mode_machine
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: tick
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
        max_serialized_size_unitree_hg__msg__IMUState(
        full_bounded, current_alignment);
    }
  }
  // member: motor_state
  {
    size_t array_size = 35;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_unitree_hg__msg__MotorState(
        full_bounded, current_alignment);
    }
  }
  // member: wireless_remote
  {
    size_t array_size = 40;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: reserve
  {
    size_t array_size = 4;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: crc
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _LowState__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_unitree_hg__msg__LowState(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_LowState = {
  "unitree_hg::msg",
  "LowState",
  _LowState__cdr_serialize,
  _LowState__cdr_deserialize,
  _LowState__get_serialized_size,
  _LowState__max_serialized_size
};

static rosidl_message_type_support_t _LowState__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_LowState,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, unitree_hg, msg, LowState)() {
  return &_LowState__type_support;
}

#if defined(__cplusplus)
}
#endif
