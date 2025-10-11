// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from unitree_hg:msg/LowState.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_HG__MSG__DETAIL__LOW_STATE__BUILDER_HPP_
#define UNITREE_HG__MSG__DETAIL__LOW_STATE__BUILDER_HPP_

#include "unitree_hg/msg/detail/low_state__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace unitree_hg
{

namespace msg
{

namespace builder
{

class Init_LowState_crc
{
public:
  explicit Init_LowState_crc(::unitree_hg::msg::LowState & msg)
  : msg_(msg)
  {}
  ::unitree_hg::msg::LowState crc(::unitree_hg::msg::LowState::_crc_type arg)
  {
    msg_.crc = std::move(arg);
    return std::move(msg_);
  }

private:
  ::unitree_hg::msg::LowState msg_;
};

class Init_LowState_reserve
{
public:
  explicit Init_LowState_reserve(::unitree_hg::msg::LowState & msg)
  : msg_(msg)
  {}
  Init_LowState_crc reserve(::unitree_hg::msg::LowState::_reserve_type arg)
  {
    msg_.reserve = std::move(arg);
    return Init_LowState_crc(msg_);
  }

private:
  ::unitree_hg::msg::LowState msg_;
};

class Init_LowState_wireless_remote
{
public:
  explicit Init_LowState_wireless_remote(::unitree_hg::msg::LowState & msg)
  : msg_(msg)
  {}
  Init_LowState_reserve wireless_remote(::unitree_hg::msg::LowState::_wireless_remote_type arg)
  {
    msg_.wireless_remote = std::move(arg);
    return Init_LowState_reserve(msg_);
  }

private:
  ::unitree_hg::msg::LowState msg_;
};

class Init_LowState_motor_state
{
public:
  explicit Init_LowState_motor_state(::unitree_hg::msg::LowState & msg)
  : msg_(msg)
  {}
  Init_LowState_wireless_remote motor_state(::unitree_hg::msg::LowState::_motor_state_type arg)
  {
    msg_.motor_state = std::move(arg);
    return Init_LowState_wireless_remote(msg_);
  }

private:
  ::unitree_hg::msg::LowState msg_;
};

class Init_LowState_imu_state
{
public:
  explicit Init_LowState_imu_state(::unitree_hg::msg::LowState & msg)
  : msg_(msg)
  {}
  Init_LowState_motor_state imu_state(::unitree_hg::msg::LowState::_imu_state_type arg)
  {
    msg_.imu_state = std::move(arg);
    return Init_LowState_motor_state(msg_);
  }

private:
  ::unitree_hg::msg::LowState msg_;
};

class Init_LowState_tick
{
public:
  explicit Init_LowState_tick(::unitree_hg::msg::LowState & msg)
  : msg_(msg)
  {}
  Init_LowState_imu_state tick(::unitree_hg::msg::LowState::_tick_type arg)
  {
    msg_.tick = std::move(arg);
    return Init_LowState_imu_state(msg_);
  }

private:
  ::unitree_hg::msg::LowState msg_;
};

class Init_LowState_mode_machine
{
public:
  explicit Init_LowState_mode_machine(::unitree_hg::msg::LowState & msg)
  : msg_(msg)
  {}
  Init_LowState_tick mode_machine(::unitree_hg::msg::LowState::_mode_machine_type arg)
  {
    msg_.mode_machine = std::move(arg);
    return Init_LowState_tick(msg_);
  }

private:
  ::unitree_hg::msg::LowState msg_;
};

class Init_LowState_mode_pr
{
public:
  explicit Init_LowState_mode_pr(::unitree_hg::msg::LowState & msg)
  : msg_(msg)
  {}
  Init_LowState_mode_machine mode_pr(::unitree_hg::msg::LowState::_mode_pr_type arg)
  {
    msg_.mode_pr = std::move(arg);
    return Init_LowState_mode_machine(msg_);
  }

private:
  ::unitree_hg::msg::LowState msg_;
};

class Init_LowState_version
{
public:
  Init_LowState_version()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LowState_mode_pr version(::unitree_hg::msg::LowState::_version_type arg)
  {
    msg_.version = std::move(arg);
    return Init_LowState_mode_pr(msg_);
  }

private:
  ::unitree_hg::msg::LowState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::unitree_hg::msg::LowState>()
{
  return unitree_hg::msg::builder::Init_LowState_version();
}

}  // namespace unitree_hg

#endif  // UNITREE_HG__MSG__DETAIL__LOW_STATE__BUILDER_HPP_
