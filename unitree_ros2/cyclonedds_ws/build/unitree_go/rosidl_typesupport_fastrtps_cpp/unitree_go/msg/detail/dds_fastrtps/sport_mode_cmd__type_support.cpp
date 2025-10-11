// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from unitree_go:msg/SportModeCmd.idl
// generated code does not contain a copyright notice
#include "unitree_go/msg/detail/sport_mode_cmd__rosidl_typesupport_fastrtps_cpp.hpp"
#include "unitree_go/msg/detail/sport_mode_cmd__struct.hpp"

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
bool cdr_serialize(
  const unitree_go::msg::PathPoint &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  unitree_go::msg::PathPoint &);
size_t get_serialized_size(
  const unitree_go::msg::PathPoint &,
  size_t current_alignment);
size_t
max_serialized_size_PathPoint(
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
  const unitree_go::msg::SportModeCmd & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: mode
  cdr << ros_message.mode;
  // Member: gait_type
  cdr << ros_message.gait_type;
  // Member: speed_level
  cdr << ros_message.speed_level;
  // Member: foot_raise_height
  cdr << ros_message.foot_raise_height;
  // Member: body_height
  cdr << ros_message.body_height;
  // Member: position
  {
    cdr << ros_message.position;
  }
  // Member: euler
  {
    cdr << ros_message.euler;
  }
  // Member: velocity
  {
    cdr << ros_message.velocity;
  }
  // Member: yaw_speed
  cdr << ros_message.yaw_speed;
  // Member: bms_cmd
  unitree_go::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.bms_cmd,
    cdr);
  // Member: path_point
  {
    for (size_t i = 0; i < 30; i++) {
      unitree_go::msg::typesupport_fastrtps_cpp::cdr_serialize(
        ros_message.path_point[i],
        cdr);
    }
  }
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_unitree_go
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  unitree_go::msg::SportModeCmd & ros_message)
{
  // Member: mode
  cdr >> ros_message.mode;

  // Member: gait_type
  cdr >> ros_message.gait_type;

  // Member: speed_level
  cdr >> ros_message.speed_level;

  // Member: foot_raise_height
  cdr >> ros_message.foot_raise_height;

  // Member: body_height
  cdr >> ros_message.body_height;

  // Member: position
  {
    cdr >> ros_message.position;
  }

  // Member: euler
  {
    cdr >> ros_message.euler;
  }

  // Member: velocity
  {
    cdr >> ros_message.velocity;
  }

  // Member: yaw_speed
  cdr >> ros_message.yaw_speed;

  // Member: bms_cmd
  unitree_go::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.bms_cmd);

  // Member: path_point
  {
    for (size_t i = 0; i < 30; i++) {
      unitree_go::msg::typesupport_fastrtps_cpp::cdr_deserialize(
        cdr,
        ros_message.path_point[i]);
    }
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_unitree_go
get_serialized_size(
  const unitree_go::msg::SportModeCmd & ros_message,
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
  // Member: gait_type
  {
    size_t item_size = sizeof(ros_message.gait_type);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: speed_level
  {
    size_t item_size = sizeof(ros_message.speed_level);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: foot_raise_height
  {
    size_t item_size = sizeof(ros_message.foot_raise_height);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: body_height
  {
    size_t item_size = sizeof(ros_message.body_height);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: position
  {
    size_t array_size = 2;
    size_t item_size = sizeof(ros_message.position[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: euler
  {
    size_t array_size = 3;
    size_t item_size = sizeof(ros_message.euler[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: velocity
  {
    size_t array_size = 2;
    size_t item_size = sizeof(ros_message.velocity[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: yaw_speed
  {
    size_t item_size = sizeof(ros_message.yaw_speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: bms_cmd

  current_alignment +=
    unitree_go::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.bms_cmd, current_alignment);
  // Member: path_point
  {
    size_t array_size = 30;

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        unitree_go::msg::typesupport_fastrtps_cpp::get_serialized_size(
        ros_message.path_point[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_unitree_go
max_serialized_size_SportModeCmd(
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

  // Member: gait_type
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: speed_level
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: foot_raise_height
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: body_height
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: position
  {
    size_t array_size = 2;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: euler
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: velocity
  {
    size_t array_size = 2;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: yaw_speed
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
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

  // Member: path_point
  {
    size_t array_size = 30;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        unitree_go::msg::typesupport_fastrtps_cpp::max_serialized_size_PathPoint(
        full_bounded, current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static bool _SportModeCmd__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const unitree_go::msg::SportModeCmd *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _SportModeCmd__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<unitree_go::msg::SportModeCmd *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _SportModeCmd__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const unitree_go::msg::SportModeCmd *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _SportModeCmd__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_SportModeCmd(full_bounded, 0);
}

static message_type_support_callbacks_t _SportModeCmd__callbacks = {
  "unitree_go::msg",
  "SportModeCmd",
  _SportModeCmd__cdr_serialize,
  _SportModeCmd__cdr_deserialize,
  _SportModeCmd__get_serialized_size,
  _SportModeCmd__max_serialized_size
};

static rosidl_message_type_support_t _SportModeCmd__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_SportModeCmd__callbacks,
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
get_message_type_support_handle<unitree_go::msg::SportModeCmd>()
{
  return &unitree_go::msg::typesupport_fastrtps_cpp::_SportModeCmd__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, unitree_go, msg, SportModeCmd)() {
  return &unitree_go::msg::typesupport_fastrtps_cpp::_SportModeCmd__handle;
}

#ifdef __cplusplus
}
#endif
