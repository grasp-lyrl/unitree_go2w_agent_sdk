// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from unitree_hg:msg/MotorCmd.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_HG__MSG__DETAIL__MOTOR_CMD__BUILDER_HPP_
#define UNITREE_HG__MSG__DETAIL__MOTOR_CMD__BUILDER_HPP_

#include "unitree_hg/msg/detail/motor_cmd__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace unitree_hg
{

namespace msg
{

namespace builder
{

class Init_MotorCmd_reserve
{
public:
  explicit Init_MotorCmd_reserve(::unitree_hg::msg::MotorCmd & msg)
  : msg_(msg)
  {}
  ::unitree_hg::msg::MotorCmd reserve(::unitree_hg::msg::MotorCmd::_reserve_type arg)
  {
    msg_.reserve = std::move(arg);
    return std::move(msg_);
  }

private:
  ::unitree_hg::msg::MotorCmd msg_;
};

class Init_MotorCmd_kd
{
public:
  explicit Init_MotorCmd_kd(::unitree_hg::msg::MotorCmd & msg)
  : msg_(msg)
  {}
  Init_MotorCmd_reserve kd(::unitree_hg::msg::MotorCmd::_kd_type arg)
  {
    msg_.kd = std::move(arg);
    return Init_MotorCmd_reserve(msg_);
  }

private:
  ::unitree_hg::msg::MotorCmd msg_;
};

class Init_MotorCmd_kp
{
public:
  explicit Init_MotorCmd_kp(::unitree_hg::msg::MotorCmd & msg)
  : msg_(msg)
  {}
  Init_MotorCmd_kd kp(::unitree_hg::msg::MotorCmd::_kp_type arg)
  {
    msg_.kp = std::move(arg);
    return Init_MotorCmd_kd(msg_);
  }

private:
  ::unitree_hg::msg::MotorCmd msg_;
};

class Init_MotorCmd_tau
{
public:
  explicit Init_MotorCmd_tau(::unitree_hg::msg::MotorCmd & msg)
  : msg_(msg)
  {}
  Init_MotorCmd_kp tau(::unitree_hg::msg::MotorCmd::_tau_type arg)
  {
    msg_.tau = std::move(arg);
    return Init_MotorCmd_kp(msg_);
  }

private:
  ::unitree_hg::msg::MotorCmd msg_;
};

class Init_MotorCmd_dq
{
public:
  explicit Init_MotorCmd_dq(::unitree_hg::msg::MotorCmd & msg)
  : msg_(msg)
  {}
  Init_MotorCmd_tau dq(::unitree_hg::msg::MotorCmd::_dq_type arg)
  {
    msg_.dq = std::move(arg);
    return Init_MotorCmd_tau(msg_);
  }

private:
  ::unitree_hg::msg::MotorCmd msg_;
};

class Init_MotorCmd_q
{
public:
  explicit Init_MotorCmd_q(::unitree_hg::msg::MotorCmd & msg)
  : msg_(msg)
  {}
  Init_MotorCmd_dq q(::unitree_hg::msg::MotorCmd::_q_type arg)
  {
    msg_.q = std::move(arg);
    return Init_MotorCmd_dq(msg_);
  }

private:
  ::unitree_hg::msg::MotorCmd msg_;
};

class Init_MotorCmd_mode
{
public:
  Init_MotorCmd_mode()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorCmd_q mode(::unitree_hg::msg::MotorCmd::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_MotorCmd_q(msg_);
  }

private:
  ::unitree_hg::msg::MotorCmd msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::unitree_hg::msg::MotorCmd>()
{
  return unitree_hg::msg::builder::Init_MotorCmd_mode();
}

}  // namespace unitree_hg

#endif  // UNITREE_HG__MSG__DETAIL__MOTOR_CMD__BUILDER_HPP_
