// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from unitree_hg:msg/IMUState.idl
// generated code does not contain a copyright notice
#include "unitree_hg/msg/detail/imu_state__rosidl_typesupport_fastrtps_cpp.hpp"
#include "unitree_hg/msg/detail/imu_state__struct.hpp"

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

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_unitree_hg
cdr_serialize(
  const unitree_hg::msg::IMUState & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: quaternion
  {
    cdr << ros_message.quaternion;
  }
  // Member: gyroscope
  {
    cdr << ros_message.gyroscope;
  }
  // Member: accelerometer
  {
    cdr << ros_message.accelerometer;
  }
  // Member: rpy
  {
    cdr << ros_message.rpy;
  }
  // Member: temperature
  cdr << ros_message.temperature;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_unitree_hg
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  unitree_hg::msg::IMUState & ros_message)
{
  // Member: quaternion
  {
    cdr >> ros_message.quaternion;
  }

  // Member: gyroscope
  {
    cdr >> ros_message.gyroscope;
  }

  // Member: accelerometer
  {
    cdr >> ros_message.accelerometer;
  }

  // Member: rpy
  {
    cdr >> ros_message.rpy;
  }

  // Member: temperature
  cdr >> ros_message.temperature;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_unitree_hg
get_serialized_size(
  const unitree_hg::msg::IMUState & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: quaternion
  {
    size_t array_size = 4;
    size_t item_size = sizeof(ros_message.quaternion[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: gyroscope
  {
    size_t array_size = 3;
    size_t item_size = sizeof(ros_message.gyroscope[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: accelerometer
  {
    size_t array_size = 3;
    size_t item_size = sizeof(ros_message.accelerometer[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: rpy
  {
    size_t array_size = 3;
    size_t item_size = sizeof(ros_message.rpy[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: temperature
  {
    size_t item_size = sizeof(ros_message.temperature);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_unitree_hg
max_serialized_size_IMUState(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: quaternion
  {
    size_t array_size = 4;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: gyroscope
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: accelerometer
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: rpy
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: temperature
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  return current_alignment - initial_alignment;
}

static bool _IMUState__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const unitree_hg::msg::IMUState *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _IMUState__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<unitree_hg::msg::IMUState *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _IMUState__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const unitree_hg::msg::IMUState *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _IMUState__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_IMUState(full_bounded, 0);
}

static message_type_support_callbacks_t _IMUState__callbacks = {
  "unitree_hg::msg",
  "IMUState",
  _IMUState__cdr_serialize,
  _IMUState__cdr_deserialize,
  _IMUState__get_serialized_size,
  _IMUState__max_serialized_size
};

static rosidl_message_type_support_t _IMUState__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_IMUState__callbacks,
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
get_message_type_support_handle<unitree_hg::msg::IMUState>()
{
  return &unitree_hg::msg::typesupport_fastrtps_cpp::_IMUState__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, unitree_hg, msg, IMUState)() {
  return &unitree_hg::msg::typesupport_fastrtps_cpp::_IMUState__handle;
}

#ifdef __cplusplus
}
#endif
