// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from unitree_go:msg/Req.idl
// generated code does not contain a copyright notice
#include "unitree_go/msg/detail/req__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "unitree_go/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "unitree_go/msg/detail/req__struct.h"
#include "unitree_go/msg/detail/req__functions.h"
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

#include "rosidl_runtime_c/string.h"  // body, uuid
#include "rosidl_runtime_c/string_functions.h"  // body, uuid

// forward declare type support functions


using _Req__ros_msg_type = unitree_go__msg__Req;

static bool _Req__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _Req__ros_msg_type * ros_message = static_cast<const _Req__ros_msg_type *>(untyped_ros_message);
  // Field name: uuid
  {
    const rosidl_runtime_c__String * str = &ros_message->uuid;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: body
  {
    const rosidl_runtime_c__String * str = &ros_message->body;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

static bool _Req__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _Req__ros_msg_type * ros_message = static_cast<_Req__ros_msg_type *>(untyped_ros_message);
  // Field name: uuid
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->uuid.data) {
      rosidl_runtime_c__String__init(&ros_message->uuid);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->uuid,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'uuid'\n");
      return false;
    }
  }

  // Field name: body
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->body.data) {
      rosidl_runtime_c__String__init(&ros_message->body);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->body,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'body'\n");
      return false;
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_unitree_go
size_t get_serialized_size_unitree_go__msg__Req(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Req__ros_msg_type * ros_message = static_cast<const _Req__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name uuid
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->uuid.size + 1);
  // field.name body
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->body.size + 1);

  return current_alignment - initial_alignment;
}

static uint32_t _Req__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_unitree_go__msg__Req(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_unitree_go
size_t max_serialized_size_unitree_go__msg__Req(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: uuid
  {
    size_t array_size = 1;

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: body
  {
    size_t array_size = 1;

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  return current_alignment - initial_alignment;
}

static size_t _Req__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_unitree_go__msg__Req(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_Req = {
  "unitree_go::msg",
  "Req",
  _Req__cdr_serialize,
  _Req__cdr_deserialize,
  _Req__get_serialized_size,
  _Req__max_serialized_size
};

static rosidl_message_type_support_t _Req__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Req,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, unitree_go, msg, Req)() {
  return &_Req__type_support;
}

#if defined(__cplusplus)
}
#endif
