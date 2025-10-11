// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from unitree_hg:msg/MainBoardState.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "unitree_hg/msg/detail/main_board_state__struct.hpp"
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

void MainBoardState_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) unitree_hg::msg::MainBoardState(_init);
}

void MainBoardState_fini_function(void * message_memory)
{
  auto typed_message = static_cast<unitree_hg::msg::MainBoardState *>(message_memory);
  typed_message->~MainBoardState();
}

size_t size_function__MainBoardState__fan_state(const void * untyped_member)
{
  (void)untyped_member;
  return 6;
}

const void * get_const_function__MainBoardState__fan_state(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<uint16_t, 6> *>(untyped_member);
  return &member[index];
}

void * get_function__MainBoardState__fan_state(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<uint16_t, 6> *>(untyped_member);
  return &member[index];
}

size_t size_function__MainBoardState__temperature(const void * untyped_member)
{
  (void)untyped_member;
  return 6;
}

const void * get_const_function__MainBoardState__temperature(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<int16_t, 6> *>(untyped_member);
  return &member[index];
}

void * get_function__MainBoardState__temperature(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<int16_t, 6> *>(untyped_member);
  return &member[index];
}

size_t size_function__MainBoardState__value(const void * untyped_member)
{
  (void)untyped_member;
  return 6;
}

const void * get_const_function__MainBoardState__value(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 6> *>(untyped_member);
  return &member[index];
}

void * get_function__MainBoardState__value(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 6> *>(untyped_member);
  return &member[index];
}

size_t size_function__MainBoardState__state(const void * untyped_member)
{
  (void)untyped_member;
  return 6;
}

const void * get_const_function__MainBoardState__state(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<uint32_t, 6> *>(untyped_member);
  return &member[index];
}

void * get_function__MainBoardState__state(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<uint32_t, 6> *>(untyped_member);
  return &member[index];
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MainBoardState_message_member_array[4] = {
  {
    "fan_state",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    6,  // array size
    false,  // is upper bound
    offsetof(unitree_hg::msg::MainBoardState, fan_state),  // bytes offset in struct
    nullptr,  // default value
    size_function__MainBoardState__fan_state,  // size() function pointer
    get_const_function__MainBoardState__fan_state,  // get_const(index) function pointer
    get_function__MainBoardState__fan_state,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "temperature",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    6,  // array size
    false,  // is upper bound
    offsetof(unitree_hg::msg::MainBoardState, temperature),  // bytes offset in struct
    nullptr,  // default value
    size_function__MainBoardState__temperature,  // size() function pointer
    get_const_function__MainBoardState__temperature,  // get_const(index) function pointer
    get_function__MainBoardState__temperature,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "value",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    6,  // array size
    false,  // is upper bound
    offsetof(unitree_hg::msg::MainBoardState, value),  // bytes offset in struct
    nullptr,  // default value
    size_function__MainBoardState__value,  // size() function pointer
    get_const_function__MainBoardState__value,  // get_const(index) function pointer
    get_function__MainBoardState__value,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "state",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    6,  // array size
    false,  // is upper bound
    offsetof(unitree_hg::msg::MainBoardState, state),  // bytes offset in struct
    nullptr,  // default value
    size_function__MainBoardState__state,  // size() function pointer
    get_const_function__MainBoardState__state,  // get_const(index) function pointer
    get_function__MainBoardState__state,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MainBoardState_message_members = {
  "unitree_hg::msg",  // message namespace
  "MainBoardState",  // message name
  4,  // number of fields
  sizeof(unitree_hg::msg::MainBoardState),
  MainBoardState_message_member_array,  // message members
  MainBoardState_init_function,  // function to initialize message memory (memory has to be allocated)
  MainBoardState_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MainBoardState_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MainBoardState_message_members,
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
get_message_type_support_handle<unitree_hg::msg::MainBoardState>()
{
  return &::unitree_hg::msg::rosidl_typesupport_introspection_cpp::MainBoardState_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, unitree_hg, msg, MainBoardState)() {
  return &::unitree_hg::msg::rosidl_typesupport_introspection_cpp::MainBoardState_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
