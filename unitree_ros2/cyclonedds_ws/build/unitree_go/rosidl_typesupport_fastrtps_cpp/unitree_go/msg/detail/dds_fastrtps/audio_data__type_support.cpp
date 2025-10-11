// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from unitree_go:msg/AudioData.idl
// generated code does not contain a copyright notice
#include "unitree_go/msg/detail/audio_data__rosidl_typesupport_fastrtps_cpp.hpp"
#include "unitree_go/msg/detail/audio_data__struct.hpp"

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
  const unitree_go::msg::AudioData & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: time_frame
  cdr << ros_message.time_frame;
  // Member: data
  {
    cdr << ros_message.data;
  }
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_unitree_go
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  unitree_go::msg::AudioData & ros_message)
{
  // Member: time_frame
  cdr >> ros_message.time_frame;

  // Member: data
  {
    cdr >> ros_message.data;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_unitree_go
get_serialized_size(
  const unitree_go::msg::AudioData & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: time_frame
  {
    size_t item_size = sizeof(ros_message.time_frame);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: data
  {
    size_t array_size = ros_message.data.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    size_t item_size = sizeof(ros_message.data[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_unitree_go
max_serialized_size_AudioData(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: time_frame
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: data
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static bool _AudioData__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const unitree_go::msg::AudioData *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _AudioData__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<unitree_go::msg::AudioData *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _AudioData__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const unitree_go::msg::AudioData *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _AudioData__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_AudioData(full_bounded, 0);
}

static message_type_support_callbacks_t _AudioData__callbacks = {
  "unitree_go::msg",
  "AudioData",
  _AudioData__cdr_serialize,
  _AudioData__cdr_deserialize,
  _AudioData__get_serialized_size,
  _AudioData__max_serialized_size
};

static rosidl_message_type_support_t _AudioData__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_AudioData__callbacks,
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
get_message_type_support_handle<unitree_go::msg::AudioData>()
{
  return &unitree_go::msg::typesupport_fastrtps_cpp::_AudioData__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, unitree_go, msg, AudioData)() {
  return &unitree_go::msg::typesupport_fastrtps_cpp::_AudioData__handle;
}

#ifdef __cplusplus
}
#endif
