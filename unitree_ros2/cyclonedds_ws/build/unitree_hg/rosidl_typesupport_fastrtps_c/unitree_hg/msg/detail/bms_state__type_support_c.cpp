// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from unitree_hg:msg/BmsState.idl
// generated code does not contain a copyright notice
#include "unitree_hg/msg/detail/bms_state__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "unitree_hg/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "unitree_hg/msg/detail/bms_state__struct.h"
#include "unitree_hg/msg/detail/bms_state__functions.h"
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


using _BmsState__ros_msg_type = unitree_hg__msg__BmsState;

static bool _BmsState__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _BmsState__ros_msg_type * ros_message = static_cast<const _BmsState__ros_msg_type *>(untyped_ros_message);
  // Field name: version_high
  {
    cdr << ros_message->version_high;
  }

  // Field name: version_low
  {
    cdr << ros_message->version_low;
  }

  // Field name: fn
  {
    cdr << ros_message->fn;
  }

  // Field name: cell_vol
  {
    size_t size = 40;
    auto array_ptr = ros_message->cell_vol;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: bmsvoltage
  {
    size_t size = 3;
    auto array_ptr = ros_message->bmsvoltage;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: current
  {
    cdr << ros_message->current;
  }

  // Field name: soc
  {
    cdr << ros_message->soc;
  }

  // Field name: soh
  {
    cdr << ros_message->soh;
  }

  // Field name: temperature
  {
    size_t size = 12;
    auto array_ptr = ros_message->temperature;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: cycle
  {
    cdr << ros_message->cycle;
  }

  // Field name: manufacturer_date
  {
    cdr << ros_message->manufacturer_date;
  }

  // Field name: bmsstate
  {
    size_t size = 5;
    auto array_ptr = ros_message->bmsstate;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: reserve
  {
    size_t size = 3;
    auto array_ptr = ros_message->reserve;
    cdr.serializeArray(array_ptr, size);
  }

  return true;
}

static bool _BmsState__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _BmsState__ros_msg_type * ros_message = static_cast<_BmsState__ros_msg_type *>(untyped_ros_message);
  // Field name: version_high
  {
    cdr >> ros_message->version_high;
  }

  // Field name: version_low
  {
    cdr >> ros_message->version_low;
  }

  // Field name: fn
  {
    cdr >> ros_message->fn;
  }

  // Field name: cell_vol
  {
    size_t size = 40;
    auto array_ptr = ros_message->cell_vol;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: bmsvoltage
  {
    size_t size = 3;
    auto array_ptr = ros_message->bmsvoltage;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: current
  {
    cdr >> ros_message->current;
  }

  // Field name: soc
  {
    cdr >> ros_message->soc;
  }

  // Field name: soh
  {
    cdr >> ros_message->soh;
  }

  // Field name: temperature
  {
    size_t size = 12;
    auto array_ptr = ros_message->temperature;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: cycle
  {
    cdr >> ros_message->cycle;
  }

  // Field name: manufacturer_date
  {
    cdr >> ros_message->manufacturer_date;
  }

  // Field name: bmsstate
  {
    size_t size = 5;
    auto array_ptr = ros_message->bmsstate;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: reserve
  {
    size_t size = 3;
    auto array_ptr = ros_message->reserve;
    cdr.deserializeArray(array_ptr, size);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_unitree_hg
size_t get_serialized_size_unitree_hg__msg__BmsState(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _BmsState__ros_msg_type * ros_message = static_cast<const _BmsState__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name version_high
  {
    size_t item_size = sizeof(ros_message->version_high);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name version_low
  {
    size_t item_size = sizeof(ros_message->version_low);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name fn
  {
    size_t item_size = sizeof(ros_message->fn);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name cell_vol
  {
    size_t array_size = 40;
    auto array_ptr = ros_message->cell_vol;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name bmsvoltage
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->bmsvoltage;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name current
  {
    size_t item_size = sizeof(ros_message->current);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name soc
  {
    size_t item_size = sizeof(ros_message->soc);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name soh
  {
    size_t item_size = sizeof(ros_message->soh);
    current_alignment += item_size +
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
  // field.name cycle
  {
    size_t item_size = sizeof(ros_message->cycle);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name manufacturer_date
  {
    size_t item_size = sizeof(ros_message->manufacturer_date);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name bmsstate
  {
    size_t array_size = 5;
    auto array_ptr = ros_message->bmsstate;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name reserve
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->reserve;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _BmsState__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_unitree_hg__msg__BmsState(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_unitree_hg
size_t max_serialized_size_unitree_hg__msg__BmsState(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: version_high
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: version_low
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: fn
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: cell_vol
  {
    size_t array_size = 40;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: bmsvoltage
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: current
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: soc
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: soh
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: temperature
  {
    size_t array_size = 12;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: cycle
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: manufacturer_date
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: bmsstate
  {
    size_t array_size = 5;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: reserve
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _BmsState__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_unitree_hg__msg__BmsState(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_BmsState = {
  "unitree_hg::msg",
  "BmsState",
  _BmsState__cdr_serialize,
  _BmsState__cdr_deserialize,
  _BmsState__get_serialized_size,
  _BmsState__max_serialized_size
};

static rosidl_message_type_support_t _BmsState__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_BmsState,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, unitree_hg, msg, BmsState)() {
  return &_BmsState__type_support;
}

#if defined(__cplusplus)
}
#endif
