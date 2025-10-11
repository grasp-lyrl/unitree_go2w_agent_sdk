// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from unitree_go:msg/LowCmd.idl
// generated code does not contain a copyright notice
#include "unitree_go/msg/detail/low_cmd__rosidl_typesupport_fastrtps_cpp.hpp"
#include "unitree_go/msg/detail/low_cmd__struct.hpp"

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
  const unitree_go::msg::MotorCmd &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  unitree_go::msg::MotorCmd &);
size_t get_serialized_size(
  const unitree_go::msg::MotorCmd &,
  size_t current_alignment);
size_t
max_serialized_size_MotorCmd(
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
bool cdr_serialize(
  const unitree_go::msg::BmsCmd &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  unitree_go::msg::BmsCmd &);
size_t get_serialized_size(
  const unitree_go::msg::BmsCmd &,
  size_t current_alignment);
size_t
max_serialized_size_BmsCmd(
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
  const unitree_go::msg::LowCmd & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: head
  {
    cdr << ros_message.head;
  }
  // Member: level_flag
  cdr << ros_message.level_flag;
  // Member: frame_reserve
  cdr << ros_message.frame_reserve;
  // Member: sn
  {
    cdr << ros_message.sn;
  }
  // Member: version
  {
    cdr << ros_message.version;
  }
  // Member: bandwidth
  cdr << ros_message.bandwidth;
  // Member: motor_cmd
  {
    for (size_t i = 0; i < 20; i++) {
      unitree_go::msg::typesupport_fastrtps_cpp::cdr_serialize(
        ros_message.motor_cmd[i],
        cdr);
    }
  }
  // Member: bms_cmd
  unitree_go::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.bms_cmd,
    cdr);
  // Member: wireless_remote
  {
    cdr << ros_message.wireless_remote;
  }
  // Member: led
  {
    cdr << ros_message.led;
  }
  // Member: fan
  {
    cdr << ros_message.fan;
  }
  // Member: gpio
  cdr << ros_message.gpio;
  // Member: reserve
  cdr << ros_message.reserve;
  // Member: crc
  cdr << ros_message.crc;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_unitree_go
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  unitree_go::msg::LowCmd & ros_message)
{
  // Member: head
  {
    cdr >> ros_message.head;
  }

  // Member: level_flag
  cdr >> ros_message.level_flag;

  // Member: frame_reserve
  cdr >> ros_message.frame_reserve;

  // Member: sn
  {
    cdr >> ros_message.sn;
  }

  // Member: version
  {
    cdr >> ros_message.version;
  }

  // Member: bandwidth
  cdr >> ros_message.bandwidth;

  // Member: motor_cmd
  {
    for (size_t i = 0; i < 20; i++) {
      unitree_go::msg::typesupport_fastrtps_cpp::cdr_deserialize(
        cdr,
        ros_message.motor_cmd[i]);
    }
  }

  // Member: bms_cmd
  unitree_go::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.bms_cmd);

  // Member: wireless_remote
  {
    cdr >> ros_message.wireless_remote;
  }

  // Member: led
  {
    cdr >> ros_message.led;
  }

  // Member: fan
  {
    cdr >> ros_message.fan;
  }

  // Member: gpio
  cdr >> ros_message.gpio;

  // Member: reserve
  cdr >> ros_message.reserve;

  // Member: crc
  cdr >> ros_message.crc;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_unitree_go
get_serialized_size(
  const unitree_go::msg::LowCmd & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: head
  {
    size_t array_size = 2;
    size_t item_size = sizeof(ros_message.head[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: level_flag
  {
    size_t item_size = sizeof(ros_message.level_flag);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: frame_reserve
  {
    size_t item_size = sizeof(ros_message.frame_reserve);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: sn
  {
    size_t array_size = 2;
    size_t item_size = sizeof(ros_message.sn[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: version
  {
    size_t array_size = 2;
    size_t item_size = sizeof(ros_message.version[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: bandwidth
  {
    size_t item_size = sizeof(ros_message.bandwidth);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: motor_cmd
  {
    size_t array_size = 20;

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        unitree_go::msg::typesupport_fastrtps_cpp::get_serialized_size(
        ros_message.motor_cmd[index], current_alignment);
    }
  }
  // Member: bms_cmd

  current_alignment +=
    unitree_go::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.bms_cmd, current_alignment);
  // Member: wireless_remote
  {
    size_t array_size = 40;
    size_t item_size = sizeof(ros_message.wireless_remote[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: led
  {
    size_t array_size = 12;
    size_t item_size = sizeof(ros_message.led[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: fan
  {
    size_t array_size = 2;
    size_t item_size = sizeof(ros_message.fan[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: gpio
  {
    size_t item_size = sizeof(ros_message.gpio);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: reserve
  {
    size_t item_size = sizeof(ros_message.reserve);
    current_alignment += item_size +
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
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_unitree_go
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


  // Member: head
  {
    size_t array_size = 2;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: level_flag
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: frame_reserve
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: sn
  {
    size_t array_size = 2;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: version
  {
    size_t array_size = 2;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: bandwidth
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: motor_cmd
  {
    size_t array_size = 20;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        unitree_go::msg::typesupport_fastrtps_cpp::max_serialized_size_MotorCmd(
        full_bounded, current_alignment);
    }
  }

  // Member: bms_cmd
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        unitree_go::msg::typesupport_fastrtps_cpp::max_serialized_size_BmsCmd(
        full_bounded, current_alignment);
    }
  }

  // Member: wireless_remote
  {
    size_t array_size = 40;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: led
  {
    size_t array_size = 12;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: fan
  {
    size_t array_size = 2;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: gpio
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: reserve
  {
    size_t array_size = 1;

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
    static_cast<const unitree_go::msg::LowCmd *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _LowCmd__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<unitree_go::msg::LowCmd *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _LowCmd__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const unitree_go::msg::LowCmd *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _LowCmd__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_LowCmd(full_bounded, 0);
}

static message_type_support_callbacks_t _LowCmd__callbacks = {
  "unitree_go::msg",
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

}  // namespace unitree_go

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_unitree_go
const rosidl_message_type_support_t *
get_message_type_support_handle<unitree_go::msg::LowCmd>()
{
  return &unitree_go::msg::typesupport_fastrtps_cpp::_LowCmd__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, unitree_go, msg, LowCmd)() {
  return &unitree_go::msg::typesupport_fastrtps_cpp::_LowCmd__handle;
}

#ifdef __cplusplus
}
#endif
