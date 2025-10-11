// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from unitree_go:msg/Go2FrontVideoData.idl
// generated code does not contain a copyright notice
#include "unitree_go/msg/detail/go2_front_video_data__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "unitree_go/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "unitree_go/msg/detail/go2_front_video_data__struct.h"
#include "unitree_go/msg/detail/go2_front_video_data__functions.h"
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

#include "rosidl_runtime_c/primitives_sequence.h"  // video180p, video360p, video720p
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // video180p, video360p, video720p

// forward declare type support functions


using _Go2FrontVideoData__ros_msg_type = unitree_go__msg__Go2FrontVideoData;

static bool _Go2FrontVideoData__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _Go2FrontVideoData__ros_msg_type * ros_message = static_cast<const _Go2FrontVideoData__ros_msg_type *>(untyped_ros_message);
  // Field name: time_frame
  {
    cdr << ros_message->time_frame;
  }

  // Field name: video720p
  {
    size_t size = ros_message->video720p.size;
    auto array_ptr = ros_message->video720p.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: video360p
  {
    size_t size = ros_message->video360p.size;
    auto array_ptr = ros_message->video360p.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: video180p
  {
    size_t size = ros_message->video180p.size;
    auto array_ptr = ros_message->video180p.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  return true;
}

static bool _Go2FrontVideoData__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _Go2FrontVideoData__ros_msg_type * ros_message = static_cast<_Go2FrontVideoData__ros_msg_type *>(untyped_ros_message);
  // Field name: time_frame
  {
    cdr >> ros_message->time_frame;
  }

  // Field name: video720p
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->video720p.data) {
      rosidl_runtime_c__uint8__Sequence__fini(&ros_message->video720p);
    }
    if (!rosidl_runtime_c__uint8__Sequence__init(&ros_message->video720p, size)) {
      return "failed to create array for field 'video720p'";
    }
    auto array_ptr = ros_message->video720p.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: video360p
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->video360p.data) {
      rosidl_runtime_c__uint8__Sequence__fini(&ros_message->video360p);
    }
    if (!rosidl_runtime_c__uint8__Sequence__init(&ros_message->video360p, size)) {
      return "failed to create array for field 'video360p'";
    }
    auto array_ptr = ros_message->video360p.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: video180p
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->video180p.data) {
      rosidl_runtime_c__uint8__Sequence__fini(&ros_message->video180p);
    }
    if (!rosidl_runtime_c__uint8__Sequence__init(&ros_message->video180p, size)) {
      return "failed to create array for field 'video180p'";
    }
    auto array_ptr = ros_message->video180p.data;
    cdr.deserializeArray(array_ptr, size);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_unitree_go
size_t get_serialized_size_unitree_go__msg__Go2FrontVideoData(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Go2FrontVideoData__ros_msg_type * ros_message = static_cast<const _Go2FrontVideoData__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name time_frame
  {
    size_t item_size = sizeof(ros_message->time_frame);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name video720p
  {
    size_t array_size = ros_message->video720p.size;
    auto array_ptr = ros_message->video720p.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name video360p
  {
    size_t array_size = ros_message->video360p.size;
    auto array_ptr = ros_message->video360p.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name video180p
  {
    size_t array_size = ros_message->video180p.size;
    auto array_ptr = ros_message->video180p.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _Go2FrontVideoData__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_unitree_go__msg__Go2FrontVideoData(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_unitree_go
size_t max_serialized_size_unitree_go__msg__Go2FrontVideoData(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: time_frame
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: video720p
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: video360p
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: video180p
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static size_t _Go2FrontVideoData__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_unitree_go__msg__Go2FrontVideoData(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_Go2FrontVideoData = {
  "unitree_go::msg",
  "Go2FrontVideoData",
  _Go2FrontVideoData__cdr_serialize,
  _Go2FrontVideoData__cdr_deserialize,
  _Go2FrontVideoData__get_serialized_size,
  _Go2FrontVideoData__max_serialized_size
};

static rosidl_message_type_support_t _Go2FrontVideoData__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Go2FrontVideoData,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, unitree_go, msg, Go2FrontVideoData)() {
  return &_Go2FrontVideoData__type_support;
}

#if defined(__cplusplus)
}
#endif
