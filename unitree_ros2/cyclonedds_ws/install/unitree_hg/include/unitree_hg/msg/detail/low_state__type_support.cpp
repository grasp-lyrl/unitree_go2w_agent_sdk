// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from unitree_hg:msg/LowState.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "unitree_hg/msg/detail/low_state__struct.hpp"
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

void LowState_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) unitree_hg::msg::LowState(_init);
}

void LowState_fini_function(void * message_memory)
{
  auto typed_message = static_cast<unitree_hg::msg::LowState *>(message_memory);
  typed_message->~LowState();
}

size_t size_function__LowState__version(const void * untyped_member)
{
  (void)untyped_member;
  return 2;
}

const void * get_const_function__LowState__version(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<uint32_t, 2> *>(untyped_member);
  return &member[index];
}

void * get_function__LowState__version(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<uint32_t, 2> *>(untyped_member);
  return &member[index];
}

size_t size_function__LowState__motor_state(const void * untyped_member)
{
  (void)untyped_member;
  return 35;
}

const void * get_const_function__LowState__motor_state(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<unitree_hg::msg::MotorState, 35> *>(untyped_member);
  return &member[index];
}

void * get_function__LowState__motor_state(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<unitree_hg::msg::MotorState, 35> *>(untyped_member);
  return &member[index];
}

size_t size_function__LowState__wireless_remote(const void * untyped_member)
{
  (void)untyped_member;
  return 40;
}

const void * get_const_function__LowState__wireless_remote(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<uint8_t, 40> *>(untyped_member);
  return &member[index];
}

void * get_function__LowState__wireless_remote(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<uint8_t, 40> *>(untyped_member);
  return &member[index];
}

size_t size_function__LowState__reserve(const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * get_const_function__LowState__reserve(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<uint32_t, 4> *>(untyped_member);
  return &member[index];
}

void * get_function__LowState__reserve(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<uint32_t, 4> *>(untyped_member);
  return &member[index];
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember LowState_message_member_array[9] = {
  {
    "version",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    2,  // array size
    false,  // is upper bound
    offsetof(unitree_hg::msg::LowState, version),  // bytes offset in struct
    nullptr,  // default value
    size_function__LowState__version,  // size() function pointer
    get_const_function__LowState__version,  // get_const(index) function pointer
    get_function__LowState__version,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "mode_pr",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_hg::msg::LowState, mode_pr),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "mode_machine",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_hg::msg::LowState, mode_machine),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "tick",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_hg::msg::LowState, tick),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "imu_state",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<unitree_hg::msg::IMUState>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_hg::msg::LowState, imu_state),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "motor_state",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<unitree_hg::msg::MotorState>(),  // members of sub message
    true,  // is array
    35,  // array size
    false,  // is upper bound
    offsetof(unitree_hg::msg::LowState, motor_state),  // bytes offset in struct
    nullptr,  // default value
    size_function__LowState__motor_state,  // size() function pointer
    get_const_function__LowState__motor_state,  // get_const(index) function pointer
    get_function__LowState__motor_state,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "wireless_remote",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    40,  // array size
    false,  // is upper bound
    offsetof(unitree_hg::msg::LowState, wireless_remote),  // bytes offset in struct
    nullptr,  // default value
    size_function__LowState__wireless_remote,  // size() function pointer
    get_const_function__LowState__wireless_remote,  // get_const(index) function pointer
    get_function__LowState__wireless_remote,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "reserve",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(unitree_hg::msg::LowState, reserve),  // bytes offset in struct
    nullptr,  // default value
    size_function__LowState__reserve,  // size() function pointer
    get_const_function__LowState__reserve,  // get_const(index) function pointer
    get_function__LowState__reserve,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "crc",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_hg::msg::LowState, crc),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers LowState_message_members = {
  "unitree_hg::msg",  // message namespace
  "LowState",  // message name
  9,  // number of fields
  sizeof(unitree_hg::msg::LowState),
  LowState_message_member_array,  // message members
  LowState_init_function,  // function to initialize message memory (memory has to be allocated)
  LowState_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t LowState_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &LowState_message_members,
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
get_message_type_support_handle<unitree_hg::msg::LowState>()
{
  return &::unitree_hg::msg::rosidl_typesupport_introspection_cpp::LowState_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, unitree_hg, msg, LowState)() {
  return &::unitree_hg::msg::rosidl_typesupport_introspection_cpp::LowState_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
