// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from unitree_hg:msg/HandState.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_HG__MSG__DETAIL__HAND_STATE__BUILDER_HPP_
#define UNITREE_HG__MSG__DETAIL__HAND_STATE__BUILDER_HPP_

#include "unitree_hg/msg/detail/hand_state__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace unitree_hg
{

namespace msg
{

namespace builder
{

class Init_HandState_reserve
{
public:
  explicit Init_HandState_reserve(::unitree_hg::msg::HandState & msg)
  : msg_(msg)
  {}
  ::unitree_hg::msg::HandState reserve(::unitree_hg::msg::HandState::_reserve_type arg)
  {
    msg_.reserve = std::move(arg);
    return std::move(msg_);
  }

private:
  ::unitree_hg::msg::HandState msg_;
};

class Init_HandState_power_a
{
public:
  explicit Init_HandState_power_a(::unitree_hg::msg::HandState & msg)
  : msg_(msg)
  {}
  Init_HandState_reserve power_a(::unitree_hg::msg::HandState::_power_a_type arg)
  {
    msg_.power_a = std::move(arg);
    return Init_HandState_reserve(msg_);
  }

private:
  ::unitree_hg::msg::HandState msg_;
};

class Init_HandState_power_v
{
public:
  explicit Init_HandState_power_v(::unitree_hg::msg::HandState & msg)
  : msg_(msg)
  {}
  Init_HandState_power_a power_v(::unitree_hg::msg::HandState::_power_v_type arg)
  {
    msg_.power_v = std::move(arg);
    return Init_HandState_power_a(msg_);
  }

private:
  ::unitree_hg::msg::HandState msg_;
};

class Init_HandState_press_sensor_state
{
public:
  explicit Init_HandState_press_sensor_state(::unitree_hg::msg::HandState & msg)
  : msg_(msg)
  {}
  Init_HandState_power_v press_sensor_state(::unitree_hg::msg::HandState::_press_sensor_state_type arg)
  {
    msg_.press_sensor_state = std::move(arg);
    return Init_HandState_power_v(msg_);
  }

private:
  ::unitree_hg::msg::HandState msg_;
};

class Init_HandState_imu_state
{
public:
  explicit Init_HandState_imu_state(::unitree_hg::msg::HandState & msg)
  : msg_(msg)
  {}
  Init_HandState_press_sensor_state imu_state(::unitree_hg::msg::HandState::_imu_state_type arg)
  {
    msg_.imu_state = std::move(arg);
    return Init_HandState_press_sensor_state(msg_);
  }

private:
  ::unitree_hg::msg::HandState msg_;
};

class Init_HandState_motor_state
{
public:
  Init_HandState_motor_state()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HandState_imu_state motor_state(::unitree_hg::msg::HandState::_motor_state_type arg)
  {
    msg_.motor_state = std::move(arg);
    return Init_HandState_imu_state(msg_);
  }

private:
  ::unitree_hg::msg::HandState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::unitree_hg::msg::HandState>()
{
  return unitree_hg::msg::builder::Init_HandState_motor_state();
}

}  // namespace unitree_hg

#endif  // UNITREE_HG__MSG__DETAIL__HAND_STATE__BUILDER_HPP_
