// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from unitree_go:msg/MotorStates.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__MOTOR_STATES__BUILDER_HPP_
#define UNITREE_GO__MSG__DETAIL__MOTOR_STATES__BUILDER_HPP_

#include "unitree_go/msg/detail/motor_states__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace unitree_go
{

namespace msg
{

namespace builder
{

class Init_MotorStates_states
{
public:
  Init_MotorStates_states()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::unitree_go::msg::MotorStates states(::unitree_go::msg::MotorStates::_states_type arg)
  {
    msg_.states = std::move(arg);
    return std::move(msg_);
  }

private:
  ::unitree_go::msg::MotorStates msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::unitree_go::msg::MotorStates>()
{
  return unitree_go::msg::builder::Init_MotorStates_states();
}

}  // namespace unitree_go

#endif  // UNITREE_GO__MSG__DETAIL__MOTOR_STATES__BUILDER_HPP_
