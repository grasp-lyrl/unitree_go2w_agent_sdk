// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from unitree_go:msg/LidarState.idl
// generated code does not contain a copyright notice
#include "unitree_go/msg/detail/lidar_state__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "unitree_go/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "unitree_go/msg/detail/lidar_state__struct.h"
#include "unitree_go/msg/detail/lidar_state__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/string.h"  // firmware_version, sdk_version, software_version
#include "rosidl_runtime_c/string_functions.h"  // firmware_version, sdk_version, software_version

// forward declare type support functions


using _LidarState__ros_msg_type = unitree_go__msg__LidarState;

static bool _LidarState__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _LidarState__ros_msg_type * ros_message = static_cast<const _LidarState__ros_msg_type *>(untyped_ros_message);
  // Field name: stamp
  {
    cdr << ros_message->stamp;
  }

  // Field name: firmware_version
  {
    const rosidl_runtime_c__String * str = &ros_message->firmware_version;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: software_version
  {
    const rosidl_runtime_c__String * str = &ros_message->software_version;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: sdk_version
  {
    const rosidl_runtime_c__String * str = &ros_message->sdk_version;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: sys_rotation_speed
  {
    cdr << ros_message->sys_rotation_speed;
  }

  // Field name: com_rotation_speed
  {
    cdr << ros_message->com_rotation_speed;
  }

  // Field name: error_state
  {
    cdr << ros_message->error_state;
  }

  // Field name: cloud_frequency
  {
    cdr << ros_message->cloud_frequency;
  }

  // Field name: cloud_packet_loss_rate
  {
    cdr << ros_message->cloud_packet_loss_rate;
  }

  // Field name: cloud_size
  {
    cdr << ros_message->cloud_size;
  }

  // Field name: cloud_scan_num
  {
    cdr << ros_message->cloud_scan_num;
  }

  // Field name: imu_frequency
  {
    cdr << ros_message->imu_frequency;
  }

  // Field name: imu_packet_loss_rate
  {
    cdr << ros_message->imu_packet_loss_rate;
  }

  // Field name: imu_rpy
  {
    size_t size = 3;
    auto array_ptr = ros_message->imu_rpy;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: serial_recv_stamp
  {
    cdr << ros_message->serial_recv_stamp;
  }

  // Field name: serial_buffer_size
  {
    cdr << ros_message->serial_buffer_size;
  }

  // Field name: serial_buffer_read
  {
    cdr << ros_message->serial_buffer_read;
  }

  return true;
}

static bool _LidarState__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _LidarState__ros_msg_type * ros_message = static_cast<_LidarState__ros_msg_type *>(untyped_ros_message);
  // Field name: stamp
  {
    cdr >> ros_message->stamp;
  }

  // Field name: firmware_version
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->firmware_version.data) {
      rosidl_runtime_c__String__init(&ros_message->firmware_version);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->firmware_version,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'firmware_version'\n");
      return false;
    }
  }

  // Field name: software_version
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->software_version.data) {
      rosidl_runtime_c__String__init(&ros_message->software_version);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->software_version,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'software_version'\n");
      return false;
    }
  }

  // Field name: sdk_version
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->sdk_version.data) {
      rosidl_runtime_c__String__init(&ros_message->sdk_version);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->sdk_version,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'sdk_version'\n");
      return false;
    }
  }

  // Field name: sys_rotation_speed
  {
    cdr >> ros_message->sys_rotation_speed;
  }

  // Field name: com_rotation_speed
  {
    cdr >> ros_message->com_rotation_speed;
  }

  // Field name: error_state
  {
    cdr >> ros_message->error_state;
  }

  // Field name: cloud_frequency
  {
    cdr >> ros_message->cloud_frequency;
  }

  // Field name: cloud_packet_loss_rate
  {
    cdr >> ros_message->cloud_packet_loss_rate;
  }

  // Field name: cloud_size
  {
    cdr >> ros_message->cloud_size;
  }

  // Field name: cloud_scan_num
  {
    cdr >> ros_message->cloud_scan_num;
  }

  // Field name: imu_frequency
  {
    cdr >> ros_message->imu_frequency;
  }

  // Field name: imu_packet_loss_rate
  {
    cdr >> ros_message->imu_packet_loss_rate;
  }

  // Field name: imu_rpy
  {
    size_t size = 3;
    auto array_ptr = ros_message->imu_rpy;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: serial_recv_stamp
  {
    cdr >> ros_message->serial_recv_stamp;
  }

  // Field name: serial_buffer_size
  {
    cdr >> ros_message->serial_buffer_size;
  }

  // Field name: serial_buffer_read
  {
    cdr >> ros_message->serial_buffer_read;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_unitree_go
size_t get_serialized_size_unitree_go__msg__LidarState(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _LidarState__ros_msg_type * ros_message = static_cast<const _LidarState__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name stamp
  {
    size_t item_size = sizeof(ros_message->stamp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name firmware_version
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->firmware_version.size + 1);
  // field.name software_version
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->software_version.size + 1);
  // field.name sdk_version
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->sdk_version.size + 1);
  // field.name sys_rotation_speed
  {
    size_t item_size = sizeof(ros_message->sys_rotation_speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name com_rotation_speed
  {
    size_t item_size = sizeof(ros_message->com_rotation_speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name error_state
  {
    size_t item_size = sizeof(ros_message->error_state);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name cloud_frequency
  {
    size_t item_size = sizeof(ros_message->cloud_frequency);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name cloud_packet_loss_rate
  {
    size_t item_size = sizeof(ros_message->cloud_packet_loss_rate);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name cloud_size
  {
    size_t item_size = sizeof(ros_message->cloud_size);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name cloud_scan_num
  {
    size_t item_size = sizeof(ros_message->cloud_scan_num);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name imu_frequency
  {
    size_t item_size = sizeof(ros_message->imu_frequency);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name imu_packet_loss_rate
  {
    size_t item_size = sizeof(ros_message->imu_packet_loss_rate);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name imu_rpy
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->imu_rpy;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name serial_recv_stamp
  {
    size_t item_size = sizeof(ros_message->serial_recv_stamp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name serial_buffer_size
  {
    size_t item_size = sizeof(ros_message->serial_buffer_size);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name serial_buffer_read
  {
    size_t item_size = sizeof(ros_message->serial_buffer_read);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _LidarState__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_unitree_go__msg__LidarState(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_unitree_go
size_t max_serialized_size_unitree_go__msg__LidarState(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: stamp
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: firmware_version
  {
    size_t array_size = 1;

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: software_version
  {
    size_t array_size = 1;

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: sdk_version
  {
    size_t array_size = 1;

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: sys_rotation_speed
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: com_rotation_speed
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: error_state
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: cloud_frequency
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: cloud_packet_loss_rate
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: cloud_size
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: cloud_scan_num
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: imu_frequency
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: imu_packet_loss_rate
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: imu_rpy
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: serial_recv_stamp
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: serial_buffer_size
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: serial_buffer_read
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _LidarState__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_unitree_go__msg__LidarState(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_LidarState = {
  "unitree_go::msg",
  "LidarState",
  _LidarState__cdr_serialize,
  _LidarState__cdr_deserialize,
  _LidarState__get_serialized_size,
  _LidarState__max_serialized_size
};

static rosidl_message_type_support_t _LidarState__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_LidarState,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, unitree_go, msg, LidarState)() {
  return &_LidarState__type_support;
}

#if defined(__cplusplus)
}
#endif
