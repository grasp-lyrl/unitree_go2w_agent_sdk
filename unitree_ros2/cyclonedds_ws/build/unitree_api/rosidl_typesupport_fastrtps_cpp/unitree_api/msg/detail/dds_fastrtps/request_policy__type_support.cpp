// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from unitree_api:msg/RequestPolicy.idl
// generated code does not contain a copyright notice
#include "unitree_api/msg/detail/request_policy__rosidl_typesupport_fastrtps_cpp.hpp"
#include "unitree_api/msg/detail/request_policy__struct.hpp"

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

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_unitree_api
cdr_serialize(
  const unitree_api::msg::RequestPolicy & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: priority
  cdr << ros_message.priority;
  // Member: noreply
  cdr << (ros_message.noreply ? true : false);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_unitree_api
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  unitree_api::msg::RequestPolicy & ros_message)
{
  // Member: priority
  cdr >> ros_message.priority;

  // Member: noreply
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.noreply = tmp ? true : false;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_unitree_api
get_serialized_size(
  const unitree_api::msg::RequestPolicy & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: priority
  {
    size_t item_size = sizeof(ros_message.priority);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: noreply
  {
    size_t item_size = sizeof(ros_message.noreply);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_unitree_api
max_serialized_size_RequestPolicy(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: priority
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: noreply
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static bool _RequestPolicy__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const unitree_api::msg::RequestPolicy *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _RequestPolicy__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<unitree_api::msg::RequestPolicy *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _RequestPolicy__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const unitree_api::msg::RequestPolicy *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _RequestPolicy__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_RequestPolicy(full_bounded, 0);
}

static message_type_support_callbacks_t _RequestPolicy__callbacks = {
  "unitree_api::msg",
  "RequestPolicy",
  _RequestPolicy__cdr_serialize,
  _RequestPolicy__cdr_deserialize,
  _RequestPolicy__get_serialized_size,
  _RequestPolicy__max_serialized_size
};

static rosidl_message_type_support_t _RequestPolicy__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_RequestPolicy__callbacks,
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
get_message_type_support_handle<unitree_api::msg::RequestPolicy>()
{
  return &unitree_api::msg::typesupport_fastrtps_cpp::_RequestPolicy__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, unitree_api, msg, RequestPolicy)() {
  return &unitree_api::msg::typesupport_fastrtps_cpp::_RequestPolicy__handle;
}

#ifdef __cplusplus
}
#endif
