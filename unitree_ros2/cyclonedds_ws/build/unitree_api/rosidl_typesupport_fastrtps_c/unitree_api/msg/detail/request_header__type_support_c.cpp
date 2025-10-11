// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from unitree_api:msg/RequestHeader.idl
// generated code does not contain a copyright notice
#include "unitree_api/msg/detail/request_header__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "unitree_api/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "unitree_api/msg/detail/request_header__struct.h"
#include "unitree_api/msg/detail/request_header__functions.h"
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

#include "unitree_api/msg/detail/request_identity__functions.h"  // identity
#include "unitree_api/msg/detail/request_lease__functions.h"  // lease
#include "unitree_api/msg/detail/request_policy__functions.h"  // policy

// forward declare type support functions
size_t get_serialized_size_unitree_api__msg__RequestIdentity(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_unitree_api__msg__RequestIdentity(
  bool & full_bounded,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, unitree_api, msg, RequestIdentity)();
size_t get_serialized_size_unitree_api__msg__RequestLease(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_unitree_api__msg__RequestLease(
  bool & full_bounded,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, unitree_api, msg, RequestLease)();
size_t get_serialized_size_unitree_api__msg__RequestPolicy(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_unitree_api__msg__RequestPolicy(
  bool & full_bounded,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, unitree_api, msg, RequestPolicy)();


using _RequestHeader__ros_msg_type = unitree_api__msg__RequestHeader;

static bool _RequestHeader__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _RequestHeader__ros_msg_type * ros_message = static_cast<const _RequestHeader__ros_msg_type *>(untyped_ros_message);
  // Field name: identity
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, unitree_api, msg, RequestIdentity
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->identity, cdr))
    {
      return false;
    }
  }

  // Field name: lease
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, unitree_api, msg, RequestLease
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->lease, cdr))
    {
      return false;
    }
  }

  // Field name: policy
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, unitree_api, msg, RequestPolicy
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->policy, cdr))
    {
      return false;
    }
  }

  return true;
}

static bool _RequestHeader__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _RequestHeader__ros_msg_type * ros_message = static_cast<_RequestHeader__ros_msg_type *>(untyped_ros_message);
  // Field name: identity
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, unitree_api, msg, RequestIdentity
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->identity))
    {
      return false;
    }
  }

  // Field name: lease
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, unitree_api, msg, RequestLease
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->lease))
    {
      return false;
    }
  }

  // Field name: policy
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, unitree_api, msg, RequestPolicy
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->policy))
    {
      return false;
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_unitree_api
size_t get_serialized_size_unitree_api__msg__RequestHeader(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _RequestHeader__ros_msg_type * ros_message = static_cast<const _RequestHeader__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name identity

  current_alignment += get_serialized_size_unitree_api__msg__RequestIdentity(
    &(ros_message->identity), current_alignment);
  // field.name lease

  current_alignment += get_serialized_size_unitree_api__msg__RequestLease(
    &(ros_message->lease), current_alignment);
  // field.name policy

  current_alignment += get_serialized_size_unitree_api__msg__RequestPolicy(
    &(ros_message->policy), current_alignment);

  return current_alignment - initial_alignment;
}

static uint32_t _RequestHeader__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_unitree_api__msg__RequestHeader(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_unitree_api
size_t max_serialized_size_unitree_api__msg__RequestHeader(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: identity
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_unitree_api__msg__RequestIdentity(
        full_bounded, current_alignment);
    }
  }
  // member: lease
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_unitree_api__msg__RequestLease(
        full_bounded, current_alignment);
    }
  }
  // member: policy
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_unitree_api__msg__RequestPolicy(
        full_bounded, current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static size_t _RequestHeader__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_unitree_api__msg__RequestHeader(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_RequestHeader = {
  "unitree_api::msg",
  "RequestHeader",
  _RequestHeader__cdr_serialize,
  _RequestHeader__cdr_deserialize,
  _RequestHeader__get_serialized_size,
  _RequestHeader__max_serialized_size
};

static rosidl_message_type_support_t _RequestHeader__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_RequestHeader,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, unitree_api, msg, RequestHeader)() {
  return &_RequestHeader__type_support;
}

#if defined(__cplusplus)
}
#endif
