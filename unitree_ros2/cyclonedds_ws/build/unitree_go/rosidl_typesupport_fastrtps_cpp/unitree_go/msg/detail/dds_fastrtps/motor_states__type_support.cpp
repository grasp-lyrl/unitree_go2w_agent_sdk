// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from unitree_go:msg/MotorStates.idl
// generated code does not contain a copyright notice
#include "unitree_go/msg/detail/motor_states__rosidl_typesupport_fastrtps_cpp.hpp"
#include "unitree_go/msg/detail/motor_states__struct.hpp"

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
bool cdr_serialize(
  const unitree_go::msg::MotorState &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  unitree_go::msg::MotorState &);
size_t get_serialized_size(
  const unitree_go::msg::MotorState &,
  size_t current_alignment);
size_t
max_serialized_size_MotorState(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace unitree_go


namespace unitree_go
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_unitree_go
cdr_serialize(
  const unitree_go::msg::MotorStates & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: states
  {
    size_t size = ros_message.states.size();
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; i++) {
      unitree_go::msg::typesupport_fastrtps_cpp::cdr_serialize(
        ros_message.states[i],
        cdr);
    }
  }
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_unitree_go
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  unitree_go::msg::MotorStates & ros_message)
{
  // Member: states
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    ros_message.states.resize(size);
    for (size_t i = 0; i < size; i++) {
      unitree_go::msg::typesupport_fastrtps_cpp::cdr_deserialize(
        cdr, ros_message.states[i]);
    }
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_unitree_go
get_serialized_size(
  const unitree_go::msg::MotorStates & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: states
  {
    size_t array_size = ros_message.states.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        unitree_go::msg::typesupport_fastrtps_cpp::get_serialized_size(
        ros_message.states[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_unitree_go
max_serialized_size_MotorStates(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: states
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        unitree_go::msg::typesupport_fastrtps_cpp::max_serialized_size_MotorState(
        full_bounded, current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static bool _MotorStates__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const unitree_go::msg::MotorStates *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _MotorStates__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<unitree_go::msg::MotorStates *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _MotorStates__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const unitree_go::msg::MotorStates *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _MotorStates__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_MotorStates(full_bounded, 0);
}

static message_type_support_callbacks_t _MotorStates__callbacks = {
  "unitree_go::msg",
  "MotorStates",
  _MotorStates__cdr_serialize,
  _MotorStates__cdr_deserialize,
  _MotorStates__get_serialized_size,
  _MotorStates__max_serialized_size
};

static rosidl_message_type_support_t _MotorStates__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_MotorStates__callbacks,
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
get_message_type_support_handle<unitree_go::msg::MotorStates>()
{
  return &unitree_go::msg::typesupport_fastrtps_cpp::_MotorStates__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, unitree_go, msg, MotorStates)() {
  return &unitree_go::msg::typesupport_fastrtps_cpp::_MotorStates__handle;
}

#ifdef __cplusplus
}
#endif
