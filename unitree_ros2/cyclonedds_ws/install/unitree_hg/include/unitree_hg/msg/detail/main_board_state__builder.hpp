// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from unitree_hg:msg/MainBoardState.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_HG__MSG__DETAIL__MAIN_BOARD_STATE__BUILDER_HPP_
#define UNITREE_HG__MSG__DETAIL__MAIN_BOARD_STATE__BUILDER_HPP_

#include "unitree_hg/msg/detail/main_board_state__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace unitree_hg
{

namespace msg
{

namespace builder
{

class Init_MainBoardState_state
{
public:
  explicit Init_MainBoardState_state(::unitree_hg::msg::MainBoardState & msg)
  : msg_(msg)
  {}
  ::unitree_hg::msg::MainBoardState state(::unitree_hg::msg::MainBoardState::_state_type arg)
  {
    msg_.state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::unitree_hg::msg::MainBoardState msg_;
};

class Init_MainBoardState_value
{
public:
  explicit Init_MainBoardState_value(::unitree_hg::msg::MainBoardState & msg)
  : msg_(msg)
  {}
  Init_MainBoardState_state value(::unitree_hg::msg::MainBoardState::_value_type arg)
  {
    msg_.value = std::move(arg);
    return Init_MainBoardState_state(msg_);
  }

private:
  ::unitree_hg::msg::MainBoardState msg_;
};

class Init_MainBoardState_temperature
{
public:
  explicit Init_MainBoardState_temperature(::unitree_hg::msg::MainBoardState & msg)
  : msg_(msg)
  {}
  Init_MainBoardState_value temperature(::unitree_hg::msg::MainBoardState::_temperature_type arg)
  {
    msg_.temperature = std::move(arg);
    return Init_MainBoardState_value(msg_);
  }

private:
  ::unitree_hg::msg::MainBoardState msg_;
};

class Init_MainBoardState_fan_state
{
public:
  Init_MainBoardState_fan_state()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MainBoardState_temperature fan_state(::unitree_hg::msg::MainBoardState::_fan_state_type arg)
  {
    msg_.fan_state = std::move(arg);
    return Init_MainBoardState_temperature(msg_);
  }

private:
  ::unitree_hg::msg::MainBoardState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::unitree_hg::msg::MainBoardState>()
{
  return unitree_hg::msg::builder::Init_MainBoardState_fan_state();
}

}  // namespace unitree_hg

#endif  // UNITREE_HG__MSG__DETAIL__MAIN_BOARD_STATE__BUILDER_HPP_
