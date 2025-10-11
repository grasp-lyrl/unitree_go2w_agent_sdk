// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from unitree_go:msg/LidarState.idl
// generated code does not contain a copyright notice
#include "unitree_go/msg/detail/lidar_state__rosidl_typesupport_fastrtps_cpp.hpp"
#include "unitree_go/msg/detail/lidar_state__struct.hpp"

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
  const unitree_go::msg::LidarState & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: stamp
  cdr << ros_message.stamp;
  // Member: firmware_version
  cdr << ros_message.firmware_version;
  // Member: software_version
  cdr << ros_message.software_version;
  // Member: sdk_version
  cdr << ros_message.sdk_version;
  // Member: sys_rotation_speed
  cdr << ros_message.sys_rotation_speed;
  // Member: com_rotation_speed
  cdr << ros_message.com_rotation_speed;
  // Member: error_state
  cdr << ros_message.error_state;
  // Member: cloud_frequency
  cdr << ros_message.cloud_frequency;
  // Member: cloud_packet_loss_rate
  cdr << ros_message.cloud_packet_loss_rate;
  // Member: cloud_size
  cdr << ros_message.cloud_size;
  // Member: cloud_scan_num
  cdr << ros_message.cloud_scan_num;
  // Member: imu_frequency
  cdr << ros_message.imu_frequency;
  // Member: imu_packet_loss_rate
  cdr << ros_message.imu_packet_loss_rate;
  // Member: imu_rpy
  {
    cdr << ros_message.imu_rpy;
  }
  // Member: serial_recv_stamp
  cdr << ros_message.serial_recv_stamp;
  // Member: serial_buffer_size
  cdr << ros_message.serial_buffer_size;
  // Member: serial_buffer_read
  cdr << ros_message.serial_buffer_read;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_unitree_go
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  unitree_go::msg::LidarState & ros_message)
{
  // Member: stamp
  cdr >> ros_message.stamp;

  // Member: firmware_version
  cdr >> ros_message.firmware_version;

  // Member: software_version
  cdr >> ros_message.software_version;

  // Member: sdk_version
  cdr >> ros_message.sdk_version;

  // Member: sys_rotation_speed
  cdr >> ros_message.sys_rotation_speed;

  // Member: com_rotation_speed
  cdr >> ros_message.com_rotation_speed;

  // Member: error_state
  cdr >> ros_message.error_state;

  // Member: cloud_frequency
  cdr >> ros_message.cloud_frequency;

  // Member: cloud_packet_loss_rate
  cdr >> ros_message.cloud_packet_loss_rate;

  // Member: cloud_size
  cdr >> ros_message.cloud_size;

  // Member: cloud_scan_num
  cdr >> ros_message.cloud_scan_num;

  // Member: imu_frequency
  cdr >> ros_message.imu_frequency;

  // Member: imu_packet_loss_rate
  cdr >> ros_message.imu_packet_loss_rate;

  // Member: imu_rpy
  {
    cdr >> ros_message.imu_rpy;
  }

  // Member: serial_recv_stamp
  cdr >> ros_message.serial_recv_stamp;

  // Member: serial_buffer_size
  cdr >> ros_message.serial_buffer_size;

  // Member: serial_buffer_read
  cdr >> ros_message.serial_buffer_read;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_unitree_go
get_serialized_size(
  const unitree_go::msg::LidarState & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: stamp
  {
    size_t item_size = sizeof(ros_message.stamp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: firmware_version
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.firmware_version.size() + 1);
  // Member: software_version
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.software_version.size() + 1);
  // Member: sdk_version
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.sdk_version.size() + 1);
  // Member: sys_rotation_speed
  {
    size_t item_size = sizeof(ros_message.sys_rotation_speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: com_rotation_speed
  {
    size_t item_size = sizeof(ros_message.com_rotation_speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: error_state
  {
    size_t item_size = sizeof(ros_message.error_state);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: cloud_frequency
  {
    size_t item_size = sizeof(ros_message.cloud_frequency);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: cloud_packet_loss_rate
  {
    size_t item_size = sizeof(ros_message.cloud_packet_loss_rate);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: cloud_size
  {
    size_t item_size = sizeof(ros_message.cloud_size);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: cloud_scan_num
  {
    size_t item_size = sizeof(ros_message.cloud_scan_num);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: imu_frequency
  {
    size_t item_size = sizeof(ros_message.imu_frequency);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: imu_packet_loss_rate
  {
    size_t item_size = sizeof(ros_message.imu_packet_loss_rate);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: imu_rpy
  {
    size_t array_size = 3;
    size_t item_size = sizeof(ros_message.imu_rpy[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: serial_recv_stamp
  {
    size_t item_size = sizeof(ros_message.serial_recv_stamp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: serial_buffer_size
  {
    size_t item_size = sizeof(ros_message.serial_buffer_size);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: serial_buffer_read
  {
    size_t item_size = sizeof(ros_message.serial_buffer_read);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_unitree_go
max_serialized_size_LidarState(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: stamp
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: firmware_version
  {
    size_t array_size = 1;

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: software_version
  {
    size_t array_size = 1;

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: sdk_version
  {
    size_t array_size = 1;

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: sys_rotation_speed
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: com_rotation_speed
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: error_state
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: cloud_frequency
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: cloud_packet_loss_rate
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: cloud_size
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: cloud_scan_num
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: imu_frequency
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: imu_packet_loss_rate
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: imu_rpy
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: serial_recv_stamp
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: serial_buffer_size
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: serial_buffer_read
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static bool _LidarState__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const unitree_go::msg::LidarState *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _LidarState__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<unitree_go::msg::LidarState *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _LidarState__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const unitree_go::msg::LidarState *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _LidarState__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_LidarState(full_bounded, 0);
}

static message_type_support_callbacks_t _LidarState__callbacks = {
  "unitree_go::msg",
  "LidarState",
  _LidarState__cdr_serialize,
  _LidarState__cdr_deserialize,
  _LidarState__get_serialized_size,
  _LidarState__max_serialized_size
};

static rosidl_message_type_support_t _LidarState__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_LidarState__callbacks,
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
get_message_type_support_handle<unitree_go::msg::LidarState>()
{
  return &unitree_go::msg::typesupport_fastrtps_cpp::_LidarState__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, unitree_go, msg, LidarState)() {
  return &unitree_go::msg::typesupport_fastrtps_cpp::_LidarState__handle;
}

#ifdef __cplusplus
}
#endif
