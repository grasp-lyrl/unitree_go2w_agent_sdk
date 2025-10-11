// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from unitree_go:msg/IMUState.idl
// generated code does not contain a copyright notice
#include "unitree_go/msg/detail/imu_state__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "unitree_go/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "unitree_go/msg/detail/imu_state__struct.h"
#include "unitree_go/msg/detail/imu_state__functions.h"
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


using _IMUState__ros_msg_type = unitree_go__msg__IMUState;

static bool _IMUState__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _IMUState__ros_msg_type * ros_message = static_cast<const _IMUState__ros_msg_type *>(untyped_ros_message);
  // Field name: quaternion
  {
    size_t size = 4;
    auto array_ptr = ros_message->quaternion;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: gyroscope
  {
    size_t size = 3;
    auto array_ptr = ros_message->gyroscope;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: accelerometer
  {
    size_t size = 3;
    auto array_ptr = ros_message->accelerometer;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: rpy
  {
    size_t size = 3;
    auto array_ptr = ros_message->rpy;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: temperature
  {
    cdr << ros_message->temperature;
  }

  return true;
}

static bool _IMUState__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _IMUState__ros_msg_type * ros_message = static_cast<_IMUState__ros_msg_type *>(untyped_ros_message);
  // Field name: quaternion
  {
    size_t size = 4;
    auto array_ptr = ros_message->quaternion;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: gyroscope
  {
    size_t size = 3;
    auto array_ptr = ros_message->gyroscope;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: accelerometer
  {
    size_t size = 3;
    auto array_ptr = ros_message->accelerometer;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: rpy
  {
    size_t size = 3;
    auto array_ptr = ros_message->rpy;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: temperature
  {
    cdr >> ros_message->temperature;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_unitree_go
size_t get_serialized_size_unitree_go__msg__IMUState(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _IMUState__ros_msg_type * ros_message = static_cast<const _IMUState__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name quaternion
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->quaternion;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name gyroscope
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->gyroscope;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name accelerometer
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->accelerometer;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name rpy
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->rpy;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name temperature
  {
    size_t item_size = sizeof(ros_message->temperature);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _IMUState__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_unitree_go__msg__IMUState(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_unitree_go
size_t max_serialized_size_unitree_go__msg__IMUState(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: quaternion
  {
    size_t array_size = 4;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: gyroscope
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: accelerometer
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: rpy
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: temperature
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static size_t _IMUState__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_unitree_go__msg__IMUState(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_IMUState = {
  "unitree_go::msg",
  "IMUState",
  _IMUState__cdr_serialize,
  _IMUState__cdr_deserialize,
  _IMUState__get_serialized_size,
  _IMUState__max_serialized_size
};

static rosidl_message_type_support_t _IMUState__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_IMUState,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, unitree_go, msg, IMUState)() {
  return &_IMUState__type_support;
}

#if defined(__cplusplus)
}
#endif
