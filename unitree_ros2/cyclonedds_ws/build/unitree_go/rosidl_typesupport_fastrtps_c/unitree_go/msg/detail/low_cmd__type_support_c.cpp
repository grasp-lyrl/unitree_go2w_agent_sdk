// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from unitree_go:msg/LowCmd.idl
// generated code does not contain a copyright notice
#include "unitree_go/msg/detail/low_cmd__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "unitree_go/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "unitree_go/msg/detail/low_cmd__struct.h"
#include "unitree_go/msg/detail/low_cmd__functions.h"
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
#include "unitree_go/msg/detail/motor_cmd__functions.h"  // motor_cmd

// forward declare type support functions
size_t get_serialized_size_unitree_go__msg__BmsCmd(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_unitree_go__msg__BmsCmd(
  bool & full_bounded,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, unitree_go, msg, BmsCmd)();
size_t get_serialized_size_unitree_go__msg__MotorCmd(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_unitree_go__msg__MotorCmd(
  bool & full_bounded,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, unitree_go, msg, MotorCmd)();


using _LowCmd__ros_msg_type = unitree_go__msg__LowCmd;

static bool _LowCmd__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _LowCmd__ros_msg_type * ros_message = static_cast<const _LowCmd__ros_msg_type *>(untyped_ros_message);
  // Field name: head
  {
    size_t size = 2;
    auto array_ptr = ros_message->head;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: level_flag
  {
    cdr << ros_message->level_flag;
  }

  // Field name: frame_reserve
  {
    cdr << ros_message->frame_reserve;
  }

  // Field name: sn
  {
    size_t size = 2;
    auto array_ptr = ros_message->sn;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: version
  {
    size_t size = 2;
    auto array_ptr = ros_message->version;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: bandwidth
  {
    cdr << ros_message->bandwidth;
  }

  // Field name: motor_cmd
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, unitree_go, msg, MotorCmd
      )()->data);
    size_t size = 20;
    auto array_ptr = ros_message->motor_cmd;
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_serialize(
          &array_ptr[i], cdr))
      {
        return false;
      }
    }
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

  // Field name: wireless_remote
  {
    size_t size = 40;
    auto array_ptr = ros_message->wireless_remote;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: led
  {
    size_t size = 12;
    auto array_ptr = ros_message->led;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: fan
  {
    size_t size = 2;
    auto array_ptr = ros_message->fan;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: gpio
  {
    cdr << ros_message->gpio;
  }

  // Field name: reserve
  {
    cdr << ros_message->reserve;
  }

  // Field name: crc
  {
    cdr << ros_message->crc;
  }

  return true;
}

static bool _LowCmd__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _LowCmd__ros_msg_type * ros_message = static_cast<_LowCmd__ros_msg_type *>(untyped_ros_message);
  // Field name: head
  {
    size_t size = 2;
    auto array_ptr = ros_message->head;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: level_flag
  {
    cdr >> ros_message->level_flag;
  }

  // Field name: frame_reserve
  {
    cdr >> ros_message->frame_reserve;
  }

  // Field name: sn
  {
    size_t size = 2;
    auto array_ptr = ros_message->sn;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: version
  {
    size_t size = 2;
    auto array_ptr = ros_message->version;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: bandwidth
  {
    cdr >> ros_message->bandwidth;
  }

  // Field name: motor_cmd
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, unitree_go, msg, MotorCmd
      )()->data);
    size_t size = 20;
    auto array_ptr = ros_message->motor_cmd;
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_deserialize(
          cdr, &array_ptr[i]))
      {
        return false;
      }
    }
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

  // Field name: wireless_remote
  {
    size_t size = 40;
    auto array_ptr = ros_message->wireless_remote;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: led
  {
    size_t size = 12;
    auto array_ptr = ros_message->led;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: fan
  {
    size_t size = 2;
    auto array_ptr = ros_message->fan;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: gpio
  {
    cdr >> ros_message->gpio;
  }

  // Field name: reserve
  {
    cdr >> ros_message->reserve;
  }

  // Field name: crc
  {
    cdr >> ros_message->crc;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_unitree_go
size_t get_serialized_size_unitree_go__msg__LowCmd(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _LowCmd__ros_msg_type * ros_message = static_cast<const _LowCmd__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name head
  {
    size_t array_size = 2;
    auto array_ptr = ros_message->head;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name level_flag
  {
    size_t item_size = sizeof(ros_message->level_flag);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name frame_reserve
  {
    size_t item_size = sizeof(ros_message->frame_reserve);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name sn
  {
    size_t array_size = 2;
    auto array_ptr = ros_message->sn;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name version
  {
    size_t array_size = 2;
    auto array_ptr = ros_message->version;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name bandwidth
  {
    size_t item_size = sizeof(ros_message->bandwidth);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name motor_cmd
  {
    size_t array_size = 20;
    auto array_ptr = ros_message->motor_cmd;

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_unitree_go__msg__MotorCmd(
        &array_ptr[index], current_alignment);
    }
  }
  // field.name bms_cmd

  current_alignment += get_serialized_size_unitree_go__msg__BmsCmd(
    &(ros_message->bms_cmd), current_alignment);
  // field.name wireless_remote
  {
    size_t array_size = 40;
    auto array_ptr = ros_message->wireless_remote;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name led
  {
    size_t array_size = 12;
    auto array_ptr = ros_message->led;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name fan
  {
    size_t array_size = 2;
    auto array_ptr = ros_message->fan;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name gpio
  {
    size_t item_size = sizeof(ros_message->gpio);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name reserve
  {
    size_t item_size = sizeof(ros_message->reserve);
    current_alignment += item_size +
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

static uint32_t _LowCmd__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_unitree_go__msg__LowCmd(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_unitree_go
size_t max_serialized_size_unitree_go__msg__LowCmd(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: head
  {
    size_t array_size = 2;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: level_flag
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: frame_reserve
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: sn
  {
    size_t array_size = 2;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: version
  {
    size_t array_size = 2;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: bandwidth
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: motor_cmd
  {
    size_t array_size = 20;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_unitree_go__msg__MotorCmd(
        full_bounded, current_alignment);
    }
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
  // member: wireless_remote
  {
    size_t array_size = 40;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: led
  {
    size_t array_size = 12;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: fan
  {
    size_t array_size = 2;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: gpio
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: reserve
  {
    size_t array_size = 1;

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

static size_t _LowCmd__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_unitree_go__msg__LowCmd(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_LowCmd = {
  "unitree_go::msg",
  "LowCmd",
  _LowCmd__cdr_serialize,
  _LowCmd__cdr_deserialize,
  _LowCmd__get_serialized_size,
  _LowCmd__max_serialized_size
};

static rosidl_message_type_support_t _LowCmd__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_LowCmd,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, unitree_go, msg, LowCmd)() {
  return &_LowCmd__type_support;
}

#if defined(__cplusplus)
}
#endif
