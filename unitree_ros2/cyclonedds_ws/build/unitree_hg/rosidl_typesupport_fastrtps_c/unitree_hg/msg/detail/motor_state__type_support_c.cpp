// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from unitree_hg:msg/MotorState.idl
// generated code does not contain a copyright notice
#include "unitree_hg/msg/detail/motor_state__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "unitree_hg/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "unitree_hg/msg/detail/motor_state__struct.h"
#include "unitree_hg/msg/detail/motor_state__functions.h"
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


using _MotorState__ros_msg_type = unitree_hg__msg__MotorState;

static bool _MotorState__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _MotorState__ros_msg_type * ros_message = static_cast<const _MotorState__ros_msg_type *>(untyped_ros_message);
  // Field name: mode
  {
    cdr << ros_message->mode;
  }

  // Field name: q
  {
    cdr << ros_message->q;
  }

  // Field name: dq
  {
    cdr << ros_message->dq;
  }

  // Field name: ddq
  {
    cdr << ros_message->ddq;
  }

  // Field name: tau_est
  {
    cdr << ros_message->tau_est;
  }

  // Field name: temperature
  {
    size_t size = 2;
    auto array_ptr = ros_message->temperature;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: vol
  {
    cdr << ros_message->vol;
  }

  // Field name: sensor
  {
    size_t size = 2;
    auto array_ptr = ros_message->sensor;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: motorstate
  {
    cdr << ros_message->motorstate;
  }

  // Field name: reserve
  {
    size_t size = 4;
    auto array_ptr = ros_message->reserve;
    cdr.serializeArray(array_ptr, size);
  }

  return true;
}

static bool _MotorState__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _MotorState__ros_msg_type * ros_message = static_cast<_MotorState__ros_msg_type *>(untyped_ros_message);
  // Field name: mode
  {
    cdr >> ros_message->mode;
  }

  // Field name: q
  {
    cdr >> ros_message->q;
  }

  // Field name: dq
  {
    cdr >> ros_message->dq;
  }

  // Field name: ddq
  {
    cdr >> ros_message->ddq;
  }

  // Field name: tau_est
  {
    cdr >> ros_message->tau_est;
  }

  // Field name: temperature
  {
    size_t size = 2;
    auto array_ptr = ros_message->temperature;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: vol
  {
    cdr >> ros_message->vol;
  }

  // Field name: sensor
  {
    size_t size = 2;
    auto array_ptr = ros_message->sensor;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: motorstate
  {
    cdr >> ros_message->motorstate;
  }

  // Field name: reserve
  {
    size_t size = 4;
    auto array_ptr = ros_message->reserve;
    cdr.deserializeArray(array_ptr, size);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_unitree_hg
size_t get_serialized_size_unitree_hg__msg__MotorState(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _MotorState__ros_msg_type * ros_message = static_cast<const _MotorState__ros_msg_type *>(untyped_ros_message);
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
  // field.name q
  {
    size_t item_size = sizeof(ros_message->q);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name dq
  {
    size_t item_size = sizeof(ros_message->dq);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ddq
  {
    size_t item_size = sizeof(ros_message->ddq);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name tau_est
  {
    size_t item_size = sizeof(ros_message->tau_est);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name temperature
  {
    size_t array_size = 2;
    auto array_ptr = ros_message->temperature;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name vol
  {
    size_t item_size = sizeof(ros_message->vol);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name sensor
  {
    size_t array_size = 2;
    auto array_ptr = ros_message->sensor;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name motorstate
  {
    size_t item_size = sizeof(ros_message->motorstate);
    current_alignment += item_size +
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

  return current_alignment - initial_alignment;
}

static uint32_t _MotorState__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_unitree_hg__msg__MotorState(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_unitree_hg
size_t max_serialized_size_unitree_hg__msg__MotorState(
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
  // member: q
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: dq
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: ddq
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: tau_est
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: temperature
  {
    size_t array_size = 2;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: vol
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: sensor
  {
    size_t array_size = 2;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: motorstate
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: reserve
  {
    size_t array_size = 4;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _MotorState__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_unitree_hg__msg__MotorState(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_MotorState = {
  "unitree_hg::msg",
  "MotorState",
  _MotorState__cdr_serialize,
  _MotorState__cdr_deserialize,
  _MotorState__get_serialized_size,
  _MotorState__max_serialized_size
};

static rosidl_message_type_support_t _MotorState__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_MotorState,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, unitree_hg, msg, MotorState)() {
  return &_MotorState__type_support;
}

#if defined(__cplusplus)
}
#endif
