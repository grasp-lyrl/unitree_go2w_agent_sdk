// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from unitree_go:msg/MotorCmds.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__MOTOR_CMDS__BUILDER_HPP_
#define UNITREE_GO__MSG__DETAIL__MOTOR_CMDS__BUILDER_HPP_

#include "unitree_go/msg/detail/motor_cmds__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace unitree_go
{

namespace msg
{

namespace builder
{

class Init_MotorCmds_cmds
{
public:
  Init_MotorCmds_cmds()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::unitree_go::msg::MotorCmds cmds(::unitree_go::msg::MotorCmds::_cmds_type arg)
  {
    msg_.cmds = std::move(arg);
    return std::move(msg_);
  }

private:
  ::unitree_go::msg::MotorCmds msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::unitree_go::msg::MotorCmds>()
{
  return unitree_go::msg::builder::Init_MotorCmds_cmds();
}

}  // namespace unitree_go

#endif  // UNITREE_GO__MSG__DETAIL__MOTOR_CMDS__BUILDER_HPP_
