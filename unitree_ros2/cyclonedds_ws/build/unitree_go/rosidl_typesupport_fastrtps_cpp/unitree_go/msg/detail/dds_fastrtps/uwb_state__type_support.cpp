// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from unitree_go:msg/UwbState.idl
// generated code does not contain a copyright notice
#include "unitree_go/msg/detail/uwb_state__rosidl_typesupport_fastrtps_cpp.hpp"
#include "unitree_go/msg/detail/uwb_state__struct.hpp"

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
  const unitree_go::msg::UwbState & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: version
  {
    cdr << ros_message.version;
  }
  // Member: channel
  cdr << ros_message.channel;
  // Member: joy_mode
  cdr << ros_message.joy_mode;
  // Member: orientation_est
  cdr << ros_message.orientation_est;
  // Member: pitch_est
  cdr << ros_message.pitch_est;
  // Member: distance_est
  cdr << ros_message.distance_est;
  // Member: yaw_est
  cdr << ros_message.yaw_est;
  // Member: tag_roll
  cdr << ros_message.tag_roll;
  // Member: tag_pitch
  cdr << ros_message.tag_pitch;
  // Member: tag_yaw
  cdr << ros_message.tag_yaw;
  // Member: base_roll
  cdr << ros_message.base_roll;
  // Member: base_pitch
  cdr << ros_message.base_pitch;
  // Member: base_yaw
  cdr << ros_message.base_yaw;
  // Member: joystick
  {
    cdr << ros_message.joystick;
  }
  // Member: error_state
  cdr << ros_message.error_state;
  // Member: buttons
  cdr << ros_message.buttons;
  // Member: enabled_from_app
  cdr << ros_message.enabled_from_app;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_unitree_go
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  unitree_go::msg::UwbState & ros_message)
{
  // Member: version
  {
    cdr >> ros_message.version;
  }

  // Member: channel
  cdr >> ros_message.channel;

  // Member: joy_mode
  cdr >> ros_message.joy_mode;

  // Member: orientation_est
  cdr >> ros_message.orientation_est;

  // Member: pitch_est
  cdr >> ros_message.pitch_est;

  // Member: distance_est
  cdr >> ros_message.distance_est;

  // Member: yaw_est
  cdr >> ros_message.yaw_est;

  // Member: tag_roll
  cdr >> ros_message.tag_roll;

  // Member: tag_pitch
  cdr >> ros_message.tag_pitch;

  // Member: tag_yaw
  cdr >> ros_message.tag_yaw;

  // Member: base_roll
  cdr >> ros_message.base_roll;

  // Member: base_pitch
  cdr >> ros_message.base_pitch;

  // Member: base_yaw
  cdr >> ros_message.base_yaw;

  // Member: joystick
  {
    cdr >> ros_message.joystick;
  }

  // Member: error_state
  cdr >> ros_message.error_state;

  // Member: buttons
  cdr >> ros_message.buttons;

  // Member: enabled_from_app
  cdr >> ros_message.enabled_from_app;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_unitree_go
get_serialized_size(
  const unitree_go::msg::UwbState & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: version
  {
    size_t array_size = 2;
    size_t item_size = sizeof(ros_message.version[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: channel
  {
    size_t item_size = sizeof(ros_message.channel);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: joy_mode
  {
    size_t item_size = sizeof(ros_message.joy_mode);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: orientation_est
  {
    size_t item_size = sizeof(ros_message.orientation_est);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: pitch_est
  {
    size_t item_size = sizeof(ros_message.pitch_est);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: distance_est
  {
    size_t item_size = sizeof(ros_message.distance_est);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: yaw_est
  {
    size_t item_size = sizeof(ros_message.yaw_est);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: tag_roll
  {
    size_t item_size = sizeof(ros_message.tag_roll);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: tag_pitch
  {
    size_t item_size = sizeof(ros_message.tag_pitch);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: tag_yaw
  {
    size_t item_size = sizeof(ros_message.tag_yaw);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: base_roll
  {
    size_t item_size = sizeof(ros_message.base_roll);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: base_pitch
  {
    size_t item_size = sizeof(ros_message.base_pitch);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: base_yaw
  {
    size_t item_size = sizeof(ros_message.base_yaw);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: joystick
  {
    size_t array_size = 2;
    size_t item_size = sizeof(ros_message.joystick[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: error_state
  {
    size_t item_size = sizeof(ros_message.error_state);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: buttons
  {
    size_t item_size = sizeof(ros_message.buttons);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: enabled_from_app
  {
    size_t item_size = sizeof(ros_message.enabled_from_app);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_unitree_go
max_serialized_size_UwbState(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: version
  {
    size_t array_size = 2;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: channel
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: joy_mode
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: orientation_est
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: pitch_est
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: distance_est
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: yaw_est
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: tag_roll
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: tag_pitch
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: tag_yaw
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: base_roll
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: base_pitch
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: base_yaw
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: joystick
  {
    size_t array_size = 2;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: error_state
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: buttons
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: enabled_from_app
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static bool _UwbState__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const unitree_go::msg::UwbState *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _UwbState__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<unitree_go::msg::UwbState *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _UwbState__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const unitree_go::msg::UwbState *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _UwbState__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_UwbState(full_bounded, 0);
}

static message_type_support_callbacks_t _UwbState__callbacks = {
  "unitree_go::msg",
  "UwbState",
  _UwbState__cdr_serialize,
  _UwbState__cdr_deserialize,
  _UwbState__get_serialized_size,
  _UwbState__max_serialized_size
};

static rosidl_message_type_support_t _UwbState__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_UwbState__callbacks,
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
get_message_type_support_handle<unitree_go::msg::UwbState>()
{
  return &unitree_go::msg::typesupport_fastrtps_cpp::_UwbState__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, unitree_go, msg, UwbState)() {
  return &unitree_go::msg::typesupport_fastrtps_cpp::_UwbState__handle;
}

#ifdef __cplusplus
}
#endif
