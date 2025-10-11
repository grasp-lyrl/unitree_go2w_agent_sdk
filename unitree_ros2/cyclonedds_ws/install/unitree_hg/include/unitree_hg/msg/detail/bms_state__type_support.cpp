// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from unitree_hg:msg/BmsState.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "unitree_hg/msg/detail/bms_state__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace unitree_hg
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void BmsState_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) unitree_hg::msg::BmsState(_init);
}

void BmsState_fini_function(void * message_memory)
{
  auto typed_message = static_cast<unitree_hg::msg::BmsState *>(message_memory);
  typed_message->~BmsState();
}

size_t size_function__BmsState__cell_vol(const void * untyped_member)
{
  (void)untyped_member;
  return 40;
}

const void * get_const_function__BmsState__cell_vol(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<uint16_t, 40> *>(untyped_member);
  return &member[index];
}

void * get_function__BmsState__cell_vol(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<uint16_t, 40> *>(untyped_member);
  return &member[index];
}

size_t size_function__BmsState__bmsvoltage(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__BmsState__bmsvoltage(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<uint32_t, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__BmsState__bmsvoltage(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<uint32_t, 3> *>(untyped_member);
  return &member[index];
}

size_t size_function__BmsState__temperature(const void * untyped_member)
{
  (void)untyped_member;
  return 12;
}

const void * get_const_function__BmsState__temperature(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<int16_t, 12> *>(untyped_member);
  return &member[index];
}

void * get_function__BmsState__temperature(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<int16_t, 12> *>(untyped_member);
  return &member[index];
}

size_t size_function__BmsState__bmsstate(const void * untyped_member)
{
  (void)untyped_member;
  return 5;
}

const void * get_const_function__BmsState__bmsstate(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<uint32_t, 5> *>(untyped_member);
  return &member[index];
}

void * get_function__BmsState__bmsstate(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<uint32_t, 5> *>(untyped_member);
  return &member[index];
}

size_t size_function__BmsState__reserve(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__BmsState__reserve(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<uint32_t, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__BmsState__reserve(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<uint32_t, 3> *>(untyped_member);
  return &member[index];
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember BmsState_message_member_array[13] = {
  {
    "version_high",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_hg::msg::BmsState, version_high),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "version_low",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_hg::msg::BmsState, version_low),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "fn",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_hg::msg::BmsState, fn),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "cell_vol",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    40,  // array size
    false,  // is upper bound
    offsetof(unitree_hg::msg::BmsState, cell_vol),  // bytes offset in struct
    nullptr,  // default value
    size_function__BmsState__cell_vol,  // size() function pointer
    get_const_function__BmsState__cell_vol,  // get_const(index) function pointer
    get_function__BmsState__cell_vol,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "bmsvoltage",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(unitree_hg::msg::BmsState, bmsvoltage),  // bytes offset in struct
    nullptr,  // default value
    size_function__BmsState__bmsvoltage,  // size() function pointer
    get_const_function__BmsState__bmsvoltage,  // get_const(index) function pointer
    get_function__BmsState__bmsvoltage,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "current",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_hg::msg::BmsState, current),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "soc",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_hg::msg::BmsState, soc),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "soh",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_hg::msg::BmsState, soh),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "temperature",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    12,  // array size
    false,  // is upper bound
    offsetof(unitree_hg::msg::BmsState, temperature),  // bytes offset in struct
    nullptr,  // default value
    size_function__BmsState__temperature,  // size() function pointer
    get_const_function__BmsState__temperature,  // get_const(index) function pointer
    get_function__BmsState__temperature,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "cycle",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_hg::msg::BmsState, cycle),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "manufacturer_date",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_hg::msg::BmsState, manufacturer_date),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "bmsstate",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    5,  // array size
    false,  // is upper bound
    offsetof(unitree_hg::msg::BmsState, bmsstate),  // bytes offset in struct
    nullptr,  // default value
    size_function__BmsState__bmsstate,  // size() function pointer
    get_const_function__BmsState__bmsstate,  // get_const(index) function pointer
    get_function__BmsState__bmsstate,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "reserve",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(unitree_hg::msg::BmsState, reserve),  // bytes offset in struct
    nullptr,  // default value
    size_function__BmsState__reserve,  // size() function pointer
    get_const_function__BmsState__reserve,  // get_const(index) function pointer
    get_function__BmsState__reserve,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers BmsState_message_members = {
  "unitree_hg::msg",  // message namespace
  "BmsState",  // message name
  13,  // number of fields
  sizeof(unitree_hg::msg::BmsState),
  BmsState_message_member_array,  // message members
  BmsState_init_function,  // function to initialize message memory (memory has to be allocated)
  BmsState_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t BmsState_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &BmsState_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace unitree_hg


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<unitree_hg::msg::BmsState>()
{
  return &::unitree_hg::msg::rosidl_typesupport_introspection_cpp::BmsState_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, unitree_hg, msg, BmsState)() {
  return &::unitree_hg::msg::rosidl_typesupport_introspection_cpp::BmsState_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
