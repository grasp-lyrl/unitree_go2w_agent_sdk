// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from unitree_hg:msg/LowCmd.idl
// generated code does not contain a copyright notice
#include "unitree_hg/msg/detail/low_cmd__rosidl_typesupport_fastrtps_cpp.hpp"
#include "unitree_hg/msg/detail/low_cmd__struct.hpp"

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
namespace unitree_hg
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const unitree_hg::msg::MotorCmd &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  unitree_hg::msg::MotorCmd &);
size_t get_serialized_size(
  const unitree_hg::msg::MotorCmd &,
  size_t current_alignment);
size_t
max_serialized_size_MotorCmd(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace unitree_hg


namespace unitree_hg
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_unitree_hg
cdr_serialize(
  const unitree_hg::msg::LowCmd & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: mode_pr
  cdr << ros_message.mode_pr;
  // Member: mode_machine
  cdr << ros_message.mode_machine;
  // Member: motor_cmd
  {
    for (size_t i = 0; i < 35; i++) {
      unitree_hg::msg::typesupport_fastrtps_cpp::cdr_serialize(
        ros_message.motor_cmd[i],
        cdr);
    }
  }
  // Member: reserve
  {
    cdr << ros_message.reserve;
  }
  // Member: crc
  cdr << ros_message.crc;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_unitree_hg
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  unitree_hg::msg::LowCmd & ros_message)
{
  // Member: mode_pr
  cdr >> ros_message.mode_pr;

  // Member: mode_machine
  cdr >> ros_message.mode_machine;

  // Member: motor_cmd
  {
    for (size_t i = 0; i < 35; i++) {
      unitree_hg::msg::typesupport_fastrtps_cpp::cdr_deserialize(
        cdr,
        ros_message.motor_cmd[i]);
    }
  }

  // Member: reserve
  {
    cdr >> ros_message.reserve;
  }

  // Member: crc
  cdr >> ros_message.crc;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_unitree_hg
get_serialized_size(
  const unitree_hg::msg::LowCmd & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: mode_pr
  {
    size_t item_size = sizeof(ros_message.mode_pr);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: mode_machine
  {
    size_t item_size = sizeof(ros_message.mode_machine);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: motor_cmd
  {
    size_t array_size = 35;

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        unitree_hg::msg::typesupport_fastrtps_cpp::get_serialized_size(
        ros_message.motor_cmd[index], current_alignment);
    }
  }
  // Member: reserve
  {
    size_t array_size = 4;
    size_t item_size = sizeof(ros_message.reserve[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: crc
  {
    size_t item_size = sizeof(ros_message.crc);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_unitree_hg
max_serialized_size_LowCmd(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: mode_pr
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: mode_machine
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: motor_cmd
  {
    size_t array_size = 35;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        unitree_hg::msg::typesupport_fastrtps_cpp::max_serialized_size_MotorCmd(
        full_bounded, current_alignment);
    }
  }

  // Member: reserve
  {
    size_t array_size = 4;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: crc
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static bool _LowCmd__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const unitree_hg::msg::LowCmd *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _LowCmd__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<unitree_hg::msg::LowCmd *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _LowCmd__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const unitree_hg::msg::LowCmd *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _LowCmd__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_LowCmd(full_bounded, 0);
}

static message_type_support_callbacks_t _LowCmd__callbacks = {
  "unitree_hg::msg",
  "LowCmd",
  _LowCmd__cdr_serialize,
  _LowCmd__cdr_deserialize,
  _LowCmd__get_serialized_size,
  _LowCmd__max_serialized_size
};

static rosidl_message_type_support_t _LowCmd__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_LowCmd__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace unitree_hg

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_unitree_hg
const rosidl_message_type_support_t *
get_message_type_support_handle<unitree_hg::msg::LowCmd>()
{
  return &unitree_hg::msg::typesupport_fastrtps_cpp::_LowCmd__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, unitree_hg, msg, LowCmd)() {
  return &unitree_hg::msg::typesupport_fastrtps_cpp::_LowCmd__handle;
}

#ifdef __cplusplus
}
#endif
