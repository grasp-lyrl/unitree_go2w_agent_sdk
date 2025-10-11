// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from unitree_hg:msg/HandState.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "unitree_hg/msg/detail/hand_state__struct.hpp"
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

void HandState_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) unitree_hg::msg::HandState(_init);
}

void HandState_fini_function(void * message_memory)
{
  auto typed_message = static_cast<unitree_hg::msg::HandState *>(message_memory);
  typed_message->~HandState();
}

size_t size_function__HandState__motor_state(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<unitree_hg::msg::MotorState> *>(untyped_member);
  return member->size();
}

const void * get_const_function__HandState__motor_state(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<unitree_hg::msg::MotorState> *>(untyped_member);
  return &member[index];
}

void * get_function__HandState__motor_state(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<unitree_hg::msg::MotorState> *>(untyped_member);
  return &member[index];
}

void resize_function__HandState__motor_state(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<unitree_hg::msg::MotorState> *>(untyped_member);
  member->resize(size);
}

size_t size_function__HandState__press_sensor_state(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<unitree_hg::msg::PressSensorState> *>(untyped_member);
  return member->size();
}

const void * get_const_function__HandState__press_sensor_state(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<unitree_hg::msg::PressSensorState> *>(untyped_member);
  return &member[index];
}

void * get_function__HandState__press_sensor_state(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<unitree_hg::msg::PressSensorState> *>(untyped_member);
  return &member[index];
}

void resize_function__HandState__press_sensor_state(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<unitree_hg::msg::PressSensorState> *>(untyped_member);
  member->resize(size);
}

size_t size_function__HandState__reserve(const void * untyped_member)
{
  (void)untyped_member;
  return 2;
}

const void * get_const_function__HandState__reserve(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<uint32_t, 2> *>(untyped_member);
  return &member[index];
}

void * get_function__HandState__reserve(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<uint32_t, 2> *>(untyped_member);
  return &member[index];
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember HandState_message_member_array[6] = {
  {
    "motor_state",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<unitree_hg::msg::MotorState>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_hg::msg::HandState, motor_state),  // bytes offset in struct
    nullptr,  // default value
    size_function__HandState__motor_state,  // size() function pointer
    get_const_function__HandState__motor_state,  // get_const(index) function pointer
    get_function__HandState__motor_state,  // get(index) function pointer
    resize_function__HandState__motor_state  // resize(index) function pointer
  },
  {
    "imu_state",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<unitree_hg::msg::IMUState>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_hg::msg::HandState, imu_state),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "press_sensor_state",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<unitree_hg::msg::PressSensorState>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_hg::msg::HandState, press_sensor_state),  // bytes offset in struct
    nullptr,  // default value
    size_function__HandState__press_sensor_state,  // size() function pointer
    get_const_function__HandState__press_sensor_state,  // get_const(index) function pointer
    get_function__HandState__press_sensor_state,  // get(index) function pointer
    resize_function__HandState__press_sensor_state  // resize(index) function pointer
  },
  {
    "power_v",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_hg::msg::HandState, power_v),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "power_a",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(unitree_hg::msg::HandState, power_a),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "reserve",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    2,  // array size
    false,  // is upper bound
    offsetof(unitree_hg::msg::HandState, reserve),  // bytes offset in struct
    nullptr,  // default value
    size_function__HandState__reserve,  // size() function pointer
    get_const_function__HandState__reserve,  // get_const(index) function pointer
    get_function__HandState__reserve,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers HandState_message_members = {
  "unitree_hg::msg",  // message namespace
  "HandState",  // message name
  6,  // number of fields
  sizeof(unitree_hg::msg::HandState),
  HandState_message_member_array,  // message members
  HandState_init_function,  // function to initialize message memory (memory has to be allocated)
  HandState_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t HandState_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &HandState_message_members,
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
get_message_type_support_handle<unitree_hg::msg::HandState>()
{
  return &::unitree_hg::msg::rosidl_typesupport_introspection_cpp::HandState_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, unitree_hg, msg, HandState)() {
  return &::unitree_hg::msg::rosidl_typesupport_introspection_cpp::HandState_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
