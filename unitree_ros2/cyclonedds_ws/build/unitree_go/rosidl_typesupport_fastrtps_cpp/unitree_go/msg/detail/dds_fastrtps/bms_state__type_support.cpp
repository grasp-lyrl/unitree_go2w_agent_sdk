// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from unitree_go:msg/BmsState.idl
// generated code does not contain a copyright notice
#include "unitree_go/msg/detail/bms_state__rosidl_typesupport_fastrtps_cpp.hpp"
#include "unitree_go/msg/detail/bms_state__struct.hpp"

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
  const unitree_go::msg::BmsState & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: version_high
  cdr << ros_message.version_high;
  // Member: version_low
  cdr << ros_message.version_low;
  // Member: status
  cdr << ros_message.status;
  // Member: soc
  cdr << ros_message.soc;
  // Member: current
  cdr << ros_message.current;
  // Member: cycle
  cdr << ros_message.cycle;
  // Member: bq_ntc
  {
    cdr << ros_message.bq_ntc;
  }
  // Member: mcu_ntc
  {
    cdr << ros_message.mcu_ntc;
  }
  // Member: cell_vol
  {
    cdr << ros_message.cell_vol;
  }
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_unitree_go
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  unitree_go::msg::BmsState & ros_message)
{
  // Member: version_high
  cdr >> ros_message.version_high;

  // Member: version_low
  cdr >> ros_message.version_low;

  // Member: status
  cdr >> ros_message.status;

  // Member: soc
  cdr >> ros_message.soc;

  // Member: current
  cdr >> ros_message.current;

  // Member: cycle
  cdr >> ros_message.cycle;

  // Member: bq_ntc
  {
    cdr >> ros_message.bq_ntc;
  }

  // Member: mcu_ntc
  {
    cdr >> ros_message.mcu_ntc;
  }

  // Member: cell_vol
  {
    cdr >> ros_message.cell_vol;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_unitree_go
get_serialized_size(
  const unitree_go::msg::BmsState & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: version_high
  {
    size_t item_size = sizeof(ros_message.version_high);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: version_low
  {
    size_t item_size = sizeof(ros_message.version_low);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: status
  {
    size_t item_size = sizeof(ros_message.status);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: soc
  {
    size_t item_size = sizeof(ros_message.soc);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: current
  {
    size_t item_size = sizeof(ros_message.current);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: cycle
  {
    size_t item_size = sizeof(ros_message.cycle);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: bq_ntc
  {
    size_t array_size = 2;
    size_t item_size = sizeof(ros_message.bq_ntc[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: mcu_ntc
  {
    size_t array_size = 2;
    size_t item_size = sizeof(ros_message.mcu_ntc[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: cell_vol
  {
    size_t array_size = 15;
    size_t item_size = sizeof(ros_message.cell_vol[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_unitree_go
max_serialized_size_BmsState(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: version_high
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: version_low
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: status
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: soc
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: current
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: cycle
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: bq_ntc
  {
    size_t array_size = 2;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: mcu_ntc
  {
    size_t array_size = 2;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: cell_vol
  {
    size_t array_size = 15;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  return current_alignment - initial_alignment;
}

static bool _BmsState__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const unitree_go::msg::BmsState *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _BmsState__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<unitree_go::msg::BmsState *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _BmsState__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const unitree_go::msg::BmsState *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _BmsState__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_BmsState(full_bounded, 0);
}

static message_type_support_callbacks_t _BmsState__callbacks = {
  "unitree_go::msg",
  "BmsState",
  _BmsState__cdr_serialize,
  _BmsState__cdr_deserialize,
  _BmsState__get_serialized_size,
  _BmsState__max_serialized_size
};

static rosidl_message_type_support_t _BmsState__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_BmsState__callbacks,
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
get_message_type_support_handle<unitree_go::msg::BmsState>()
{
  return &unitree_go::msg::typesupport_fastrtps_cpp::_BmsState__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, unitree_go, msg, BmsState)() {
  return &unitree_go::msg::typesupport_fastrtps_cpp::_BmsState__handle;
}

#ifdef __cplusplus
}
#endif
