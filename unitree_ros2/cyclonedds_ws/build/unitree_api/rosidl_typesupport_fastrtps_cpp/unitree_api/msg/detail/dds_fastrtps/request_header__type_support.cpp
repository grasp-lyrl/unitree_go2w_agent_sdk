// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from unitree_api:msg/RequestHeader.idl
// generated code does not contain a copyright notice
#include "unitree_api/msg/detail/request_header__rosidl_typesupport_fastrtps_cpp.hpp"
#include "unitree_api/msg/detail/request_header__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace unitree_api
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const unitree_api::msg::RequestIdentity &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  unitree_api::msg::RequestIdentity &);
size_t get_serialized_size(
  const unitree_api::msg::RequestIdentity &,
  size_t current_alignment);
size_t
max_serialized_size_RequestIdentity(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace unitree_api

namespace unitree_api
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const unitree_api::msg::RequestLease &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  unitree_api::msg::RequestLease &);
size_t get_serialized_size(
  const unitree_api::msg::RequestLease &,
  size_t current_alignment);
size_t
max_serialized_size_RequestLease(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace unitree_api

namespace unitree_api
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const unitree_api::msg::RequestPolicy &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  unitree_api::msg::RequestPolicy &);
size_t get_serialized_size(
  const unitree_api::msg::RequestPolicy &,
  size_t current_alignment);
size_t
max_serialized_size_RequestPolicy(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace unitree_api


namespace unitree_api
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_unitree_api
cdr_serialize(
  const unitree_api::msg::RequestHeader & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: identity
  unitree_api::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.identity,
    cdr);
  // Member: lease
  unitree_api::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.lease,
    cdr);
  // Member: policy
  unitree_api::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.policy,
    cdr);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_unitree_api
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  unitree_api::msg::RequestHeader & ros_message)
{
  // Member: identity
  unitree_api::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.identity);

  // Member: lease
  unitree_api::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.lease);

  // Member: policy
  unitree_api::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.policy);

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_unitree_api
get_serialized_size(
  const unitree_api::msg::RequestHeader & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: identity

  current_alignment +=
    unitree_api::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.identity, current_alignment);
  // Member: lease

  current_alignment +=
    unitree_api::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.lease, current_alignment);
  // Member: policy

  current_alignment +=
    unitree_api::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.policy, current_alignment);

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_unitree_api
max_serialized_size_RequestHeader(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: identity
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        unitree_api::msg::typesupport_fastrtps_cpp::max_serialized_size_RequestIdentity(
        full_bounded, current_alignment);
    }
  }

  // Member: lease
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        unitree_api::msg::typesupport_fastrtps_cpp::max_serialized_size_RequestLease(
        full_bounded, current_alignment);
    }
  }

  // Member: policy
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        unitree_api::msg::typesupport_fastrtps_cpp::max_serialized_size_RequestPolicy(
        full_bounded, current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static bool _RequestHeader__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const unitree_api::msg::RequestHeader *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _RequestHeader__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<unitree_api::msg::RequestHeader *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _RequestHeader__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const unitree_api::msg::RequestHeader *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _RequestHeader__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_RequestHeader(full_bounded, 0);
}

static message_type_support_callbacks_t _RequestHeader__callbacks = {
  "unitree_api::msg",
  "RequestHeader",
  _RequestHeader__cdr_serialize,
  _RequestHeader__cdr_deserialize,
  _RequestHeader__get_serialized_size,
  _RequestHeader__max_serialized_size
};

static rosidl_message_type_support_t _RequestHeader__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_RequestHeader__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace unitree_api

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_unitree_api
const rosidl_message_type_support_t *
get_message_type_support_handle<unitree_api::msg::RequestHeader>()
{
  return &unitree_api::msg::typesupport_fastrtps_cpp::_RequestHeader__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, unitree_api, msg, RequestHeader)() {
  return &unitree_api::msg::typesupport_fastrtps_cpp::_RequestHeader__handle;
}

#ifdef __cplusplus
}
#endif
