// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from unitree_hg:msg/MotorState.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_HG__MSG__DETAIL__MOTOR_STATE__BUILDER_HPP_
#define UNITREE_HG__MSG__DETAIL__MOTOR_STATE__BUILDER_HPP_

#include "unitree_hg/msg/detail/motor_state__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace unitree_hg
{

namespace msg
{

namespace builder
{

class Init_MotorState_reserve
{
public:
  explicit Init_MotorState_reserve(::unitree_hg::msg::MotorState & msg)
  : msg_(msg)
  {}
  ::unitree_hg::msg::MotorState reserve(::unitree_hg::msg::MotorState::_reserve_type arg)
  {
    msg_.reserve = std::move(arg);
    return std::move(msg_);
  }

private:
  ::unitree_hg::msg::MotorState msg_;
};

class Init_MotorState_motorstate
{
public:
  explicit Init_MotorState_motorstate(::unitree_hg::msg::MotorState & msg)
  : msg_(msg)
  {}
  Init_MotorState_reserve motorstate(::unitree_hg::msg::MotorState::_motorstate_type arg)
  {
    msg_.motorstate = std::move(arg);
    return Init_MotorState_reserve(msg_);
  }

private:
  ::unitree_hg::msg::MotorState msg_;
};

class Init_MotorState_sensor
{
public:
  explicit Init_MotorState_sensor(::unitree_hg::msg::MotorState & msg)
  : msg_(msg)
  {}
  Init_MotorState_motorstate sensor(::unitree_hg::msg::MotorState::_sensor_type arg)
  {
    msg_.sensor = std::move(arg);
    return Init_MotorState_motorstate(msg_);
  }

private:
  ::unitree_hg::msg::MotorState msg_;
};

class Init_MotorState_vol
{
public:
  explicit Init_MotorState_vol(::unitree_hg::msg::MotorState & msg)
  : msg_(msg)
  {}
  Init_MotorState_sensor vol(::unitree_hg::msg::MotorState::_vol_type arg)
  {
    msg_.vol = std::move(arg);
    return Init_MotorState_sensor(msg_);
  }

private:
  ::unitree_hg::msg::MotorState msg_;
};

class Init_MotorState_temperature
{
public:
  explicit Init_MotorState_temperature(::unitree_hg::msg::MotorState & msg)
  : msg_(msg)
  {}
  Init_MotorState_vol temperature(::unitree_hg::msg::MotorState::_temperature_type arg)
  {
    msg_.temperature = std::move(arg);
    return Init_MotorState_vol(msg_);
  }

private:
  ::unitree_hg::msg::MotorState msg_;
};

class Init_MotorState_tau_est
{
public:
  explicit Init_MotorState_tau_est(::unitree_hg::msg::MotorState & msg)
  : msg_(msg)
  {}
  Init_MotorState_temperature tau_est(::unitree_hg::msg::MotorState::_tau_est_type arg)
  {
    msg_.tau_est = std::move(arg);
    return Init_MotorState_temperature(msg_);
  }

private:
  ::unitree_hg::msg::MotorState msg_;
};

class Init_MotorState_ddq
{
public:
  explicit Init_MotorState_ddq(::unitree_hg::msg::MotorState & msg)
  : msg_(msg)
  {}
  Init_MotorState_tau_est ddq(::unitree_hg::msg::MotorState::_ddq_type arg)
  {
    msg_.ddq = std::move(arg);
    return Init_MotorState_tau_est(msg_);
  }

private:
  ::unitree_hg::msg::MotorState msg_;
};

class Init_MotorState_dq
{
public:
  explicit Init_MotorState_dq(::unitree_hg::msg::MotorState & msg)
  : msg_(msg)
  {}
  Init_MotorState_ddq dq(::unitree_hg::msg::MotorState::_dq_type arg)
  {
    msg_.dq = std::move(arg);
    return Init_MotorState_ddq(msg_);
  }

private:
  ::unitree_hg::msg::MotorState msg_;
};

class Init_MotorState_q
{
public:
  explicit Init_MotorState_q(::unitree_hg::msg::MotorState & msg)
  : msg_(msg)
  {}
  Init_MotorState_dq q(::unitree_hg::msg::MotorState::_q_type arg)
  {
    msg_.q = std::move(arg);
    return Init_MotorState_dq(msg_);
  }

private:
  ::unitree_hg::msg::MotorState msg_;
};

class Init_MotorState_mode
{
public:
  Init_MotorState_mode()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorState_q mode(::unitree_hg::msg::MotorState::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_MotorState_q(msg_);
  }

private:
  ::unitree_hg::msg::MotorState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::unitree_hg::msg::MotorState>()
{
  return unitree_hg::msg::builder::Init_MotorState_mode();
}

}  // namespace unitree_hg

#endif  // UNITREE_HG__MSG__DETAIL__MOTOR_STATE__BUILDER_HPP_
