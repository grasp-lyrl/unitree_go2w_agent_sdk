// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from unitree_go:msg/InterfaceConfig.idl
// generated code does not contain a copyright notice
#include "unitree_go/msg/detail/interface_config__rosidl_typesupport_fastrtps_cpp.hpp"
#include "unitree_go/msg/detail/interface_config__struct.hpp"

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

namespace unitree_go
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_unitree_go
cdr_serialize(
  const unitree_go::msg::InterfaceConfig & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: mode
  cdr << ros_message.mode;
  // Member: value
  cdr << ros_message.value;
  // Member: reserve
  {
    cdr << ros_message.reserve;
  }
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_unitree_go
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  unitree_go::msg::InterfaceConfig & ros_message)
{
  // Member: mode
  cdr >> ros_message.mode;

  // Member: value
  cdr >> ros_message.value;

  // Member: reserve
  {
    cdr >> ros_message.reserve;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_unitree_go
get_serialized_size(
  const unitree_go::msg::InterfaceConfig & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: mode
  {
    size_t item_size = sizeof(ros_message.mode);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: value
  {
    size_t item_size = sizeof(ros_message.value);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: reserve
  {
    size_t array_size = 2;
    size_t item_size = sizeof(ros_message.reserve[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_unitree_go
max_serialized_size_InterfaceConfig(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: mode
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: value
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: reserve
  {
    size_t array_size = 2;

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static bool _InterfaceConfig__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const unitree_go::msg::InterfaceConfig *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _InterfaceConfig__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<unitree_go::msg::InterfaceConfig *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _InterfaceConfig__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const unitree_go::msg::InterfaceConfig *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _InterfaceConfig__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_InterfaceConfig(full_bounded, 0);
}

static message_type_support_callbacks_t _InterfaceConfig__callbacks = {
  "unitree_go::msg",
  "InterfaceConfig",
  _InterfaceConfig__cdr_serialize,
  _InterfaceConfig__cdr_deserialize,
  _InterfaceConfig__get_serialized_size,
  _InterfaceConfig__max_serialized_size
};

static rosidl_message_type_support_t _InterfaceConfig__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_InterfaceConfig__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace unitree_go

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_unitree_go
const rosidl_message_type_support_t *
get_message_type_support_handle<unitree_go::msg::InterfaceConfig>()
{
  return &unitree_go::msg::typesupport_fastrtps_cpp::_InterfaceConfig__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, unitree_go, msg, InterfaceConfig)() {
  return &unitree_go::msg::typesupport_fastrtps_cpp::_InterfaceConfig__handle;
}

#ifdef __cplusplus
}
#endif
