// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from unitree_hg:msg/HandState.idl
// generated code does not contain a copyright notice
#include "unitree_hg/msg/detail/hand_state__rosidl_typesupport_fastrtps_cpp.hpp"
#include "unitree_hg/msg/detail/hand_state__struct.hpp"

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
  const unitree_hg::msg::MotorState &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  unitree_hg::msg::MotorState &);
size_t get_serialized_size(
  const unitree_hg::msg::MotorState &,
  size_t current_alignment);
size_t
max_serialized_size_MotorState(
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
bool cdr_serialize(
  const unitree_hg::msg::IMUState &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  unitree_hg::msg::IMUState &);
size_t get_serialized_size(
  const unitree_hg::msg::IMUState &,
  size_t current_alignment);
size_t
max_serialized_size_IMUState(
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
bool cdr_serialize(
  const unitree_hg::msg::PressSensorState &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  unitree_hg::msg::PressSensorState &);
size_t get_serialized_size(
  const unitree_hg::msg::PressSensorState &,
  size_t current_alignment);
size_t
max_serialized_size_PressSensorState(
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
  const unitree_hg::msg::HandState & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: motor_state
  {
    size_t size = ros_message.motor_state.size();
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; i++) {
      unitree_hg::msg::typesupport_fastrtps_cpp::cdr_serialize(
        ros_message.motor_state[i],
        cdr);
    }
  }
  // Member: imu_state
  unitree_hg::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.imu_state,
    cdr);
  // Member: press_sensor_state
  {
    size_t size = ros_message.press_sensor_state.size();
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; i++) {
      unitree_hg::msg::typesupport_fastrtps_cpp::cdr_serialize(
        ros_message.press_sensor_state[i],
        cdr);
    }
  }
  // Member: power_v
  cdr << ros_message.power_v;
  // Member: power_a
  cdr << ros_message.power_a;
  // Member: reserve
  {
    cdr << ros_message.reserve;
  }
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_unitree_hg
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  unitree_hg::msg::HandState & ros_message)
{
  // Member: motor_state
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    ros_message.motor_state.resize(size);
    for (size_t i = 0; i < size; i++) {
      unitree_hg::msg::typesupport_fastrtps_cpp::cdr_deserialize(
        cdr, ros_message.motor_state[i]);
    }
  }

  // Member: imu_state
  unitree_hg::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.imu_state);

  // Member: press_sensor_state
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    ros_message.press_sensor_state.resize(size);
    for (size_t i = 0; i < size; i++) {
      unitree_hg::msg::typesupport_fastrtps_cpp::cdr_deserialize(
        cdr, ros_message.press_sensor_state[i]);
    }
  }

  // Member: power_v
  cdr >> ros_message.power_v;

  // Member: power_a
  cdr >> ros_message.power_a;

  // Member: reserve
  {
    cdr >> ros_message.reserve;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_unitree_hg
get_serialized_size(
  const unitree_hg::msg::HandState & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: motor_state
  {
    size_t array_size = ros_message.motor_state.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        unitree_hg::msg::typesupport_fastrtps_cpp::get_serialized_size(
        ros_message.motor_state[index], current_alignment);
    }
  }
  // Member: imu_state

  current_alignment +=
    unitree_hg::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.imu_state, current_alignment);
  // Member: press_sensor_state
  {
    size_t array_size = ros_message.press_sensor_state.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        unitree_hg::msg::typesupport_fastrtps_cpp::get_serialized_size(
        ros_message.press_sensor_state[index], current_alignment);
    }
  }
  // Member: power_v
  {
    size_t item_size = sizeof(ros_message.power_v);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: power_a
  {
    size_t item_size = sizeof(ros_message.power_a);
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
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_unitree_hg
max_serialized_size_HandState(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: motor_state
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        unitree_hg::msg::typesupport_fastrtps_cpp::max_serialized_size_MotorState(
        full_bounded, current_alignment);
    }
  }

  // Member: imu_state
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        unitree_hg::msg::typesupport_fastrtps_cpp::max_serialized_size_IMUState(
        full_bounded, current_alignment);
    }
  }

  // Member: press_sensor_state
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        unitree_hg::msg::typesupport_fastrtps_cpp::max_serialized_size_PressSensorState(
        full_bounded, current_alignment);
    }
  }

  // Member: power_v
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: power_a
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: reserve
  {
    size_t array_size = 2;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static bool _HandState__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const unitree_hg::msg::HandState *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _HandState__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<unitree_hg::msg::HandState *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _HandState__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const unitree_hg::msg::HandState *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _HandState__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_HandState(full_bounded, 0);
}

static message_type_support_callbacks_t _HandState__callbacks = {
  "unitree_hg::msg",
  "HandState",
  _HandState__cdr_serialize,
  _HandState__cdr_deserialize,
  _HandState__get_serialized_size,
  _HandState__max_serialized_size
};

static rosidl_message_type_support_t _HandState__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_HandState__callbacks,
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
get_message_type_support_handle<unitree_hg::msg::HandState>()
{
  return &unitree_hg::msg::typesupport_fastrtps_cpp::_HandState__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, unitree_hg, msg, HandState)() {
  return &unitree_hg::msg::typesupport_fastrtps_cpp::_HandState__handle;
}

#ifdef __cplusplus
}
#endif
