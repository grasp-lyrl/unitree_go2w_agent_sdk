// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from unitree_api:msg/Response.idl
// generated code does not contain a copyright notice
#include "unitree_api/msg/detail/response__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "unitree_api/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "unitree_api/msg/detail/response__struct.h"
#include "unitree_api/msg/detail/response__functions.h"
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

#include "rosidl_runtime_c/primitives_sequence.h"  // binary
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // binary
#include "rosidl_runtime_c/string.h"  // data
#include "rosidl_runtime_c/string_functions.h"  // data
#include "unitree_api/msg/detail/response_header__functions.h"  // header

// forward declare type support functions
size_t get_serialized_size_unitree_api__msg__ResponseHeader(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_unitree_api__msg__ResponseHeader(
  bool & full_bounded,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, unitree_api, msg, ResponseHeader)();


using _Response__ros_msg_type = unitree_api__msg__Response;

static bool _Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _Response__ros_msg_type * ros_message = static_cast<const _Response__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, unitree_api, msg, ResponseHeader
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->header, cdr))
    {
      return false;
    }
  }

  // Field name: data
  {
    const rosidl_runtime_c__String * str = &ros_message->data;
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

  // Field name: binary
  {
    size_t size = ros_message->binary.size;
    auto array_ptr = ros_message->binary.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  return true;
}

static bool _Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _Response__ros_msg_type * ros_message = static_cast<_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, unitree_api, msg, ResponseHeader
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->header))
    {
      return false;
    }
  }

  // Field name: data
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->data.data) {
      rosidl_runtime_c__String__init(&ros_message->data);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->data,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'data'\n");
      return false;
    }
  }

  // Field name: binary
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->binary.data) {
      rosidl_runtime_c__int8__Sequence__fini(&ros_message->binary);
    }
    if (!rosidl_runtime_c__int8__Sequence__init(&ros_message->binary, size)) {
      return "failed to create array for field 'binary'";
    }
    auto array_ptr = ros_message->binary.data;
    cdr.deserializeArray(array_ptr, size);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_unitree_api
size_t get_serialized_size_unitree_api__msg__Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Response__ros_msg_type * ros_message = static_cast<const _Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name header

  current_alignment += get_serialized_size_unitree_api__msg__ResponseHeader(
    &(ros_message->header), current_alignment);
  // field.name data
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->data.size + 1);
  // field.name binary
  {
    size_t array_size = ros_message->binary.size;
    auto array_ptr = ros_message->binary.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_unitree_api__msg__Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_unitree_api
size_t max_serialized_size_unitree_api__msg__Response(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: header
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_unitree_api__msg__ResponseHeader(
        full_bounded, current_alignment);
    }
  }
  // member: data
  {
    size_t array_size = 1;

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: binary
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static size_t _Response__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_unitree_api__msg__Response(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_Response = {
  "unitree_api::msg",
  "Response",
  _Response__cdr_serialize,
  _Response__cdr_deserialize,
  _Response__get_serialized_size,
  _Response__max_serialized_size
};

static rosidl_message_type_support_t _Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, unitree_api, msg, Response)() {
  return &_Response__type_support;
}

#if defined(__cplusplus)
}
#endif
