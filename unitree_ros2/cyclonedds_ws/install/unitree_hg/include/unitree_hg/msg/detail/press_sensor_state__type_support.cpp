// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from unitree_hg:msg/PressSensorState.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "unitree_hg/msg/detail/press_sensor_state__struct.hpp"
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

void PressSensorState_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) unitree_hg::msg::PressSensorState(_init);
}

void PressSensorState_fini_function(void * message_memory)
{
  auto typed_message = static_cast<unitree_hg::msg::PressSensorState *>(message_memory);
  typed_message->~PressSensorState();
}

size_t size_function__PressSensorState__pressure(const void * untyped_member)
{
  (void)untyped_member;
  return 12;
}

const void * get_const_function__PressSensorState__pressure(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 12> *>(untyped_member);
  return &member[index];
}

void * get_function__PressSensorState__pressure(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 12> *>(untyped_member);
  return &member[index];
}

size_t size_function__PressSensorState__temperature(const void * untyped_member)
{
  (void)untyped_member;
  return 12;
}

const void * get_const_function__PressSensorState__temperature(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 12> *>(untyped_member);
  return &member[index];
}

void * get_function__PressSensorState__temperature(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 12> *>(untyped_member);
  return &member[index];
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember PressSensorState_message_member_array[2] = {
  {
    "pressure",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    12,  // array size
    false,  // is upper bound
    offsetof(unitree_hg::msg::PressSensorState, pressure),  // bytes offset in struct
    nullptr,  // default value
    size_function__PressSensorState__pressure,  // size() function pointer
    get_const_function__PressSensorState__pressure,  // get_const(index) function pointer
    get_function__PressSensorState__pressure,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "temperature",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    12,  // array size
    false,  // is upper bound
    offsetof(unitree_hg::msg::PressSensorState, temperature),  // bytes offset in struct
    nullptr,  // default value
    size_function__PressSensorState__temperature,  // size() function pointer
    get_const_function__PressSensorState__temperature,  // get_const(index) function pointer
    get_function__PressSensorState__temperature,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers PressSensorState_message_members = {
  "unitree_hg::msg",  // message namespace
  "PressSensorState",  // message name
  2,  // number of fields
  sizeof(unitree_hg::msg::PressSensorState),
  PressSensorState_message_member_array,  // message members
  PressSensorState_init_function,  // function to initialize message memory (memory has to be allocated)
  PressSensorState_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t PressSensorState_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &PressSensorState_message_members,
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
get_message_type_support_handle<unitree_hg::msg::PressSensorState>()
{
  return &::unitree_hg::msg::rosidl_typesupport_introspection_cpp::PressSensorState_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, unitree_hg, msg, PressSensorState)() {
  return &::unitree_hg::msg::rosidl_typesupport_introspection_cpp::PressSensorState_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
