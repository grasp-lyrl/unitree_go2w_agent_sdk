// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from unitree_hg:msg/LowCmd.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_HG__MSG__DETAIL__LOW_CMD__BUILDER_HPP_
#define UNITREE_HG__MSG__DETAIL__LOW_CMD__BUILDER_HPP_

#include "unitree_hg/msg/detail/low_cmd__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace unitree_hg
{

namespace msg
{

namespace builder
{

class Init_LowCmd_crc
{
public:
  explicit Init_LowCmd_crc(::unitree_hg::msg::LowCmd & msg)
  : msg_(msg)
  {}
  ::unitree_hg::msg::LowCmd crc(::unitree_hg::msg::LowCmd::_crc_type arg)
  {
    msg_.crc = std::move(arg);
    return std::move(msg_);
  }

private:
  ::unitree_hg::msg::LowCmd msg_;
};

class Init_LowCmd_reserve
{
public:
  explicit Init_LowCmd_reserve(::unitree_hg::msg::LowCmd & msg)
  : msg_(msg)
  {}
  Init_LowCmd_crc reserve(::unitree_hg::msg::LowCmd::_reserve_type arg)
  {
    msg_.reserve = std::move(arg);
    return Init_LowCmd_crc(msg_);
  }

private:
  ::unitree_hg::msg::LowCmd msg_;
};

class Init_LowCmd_motor_cmd
{
public:
  explicit Init_LowCmd_motor_cmd(::unitree_hg::msg::LowCmd & msg)
  : msg_(msg)
  {}
  Init_LowCmd_reserve motor_cmd(::unitree_hg::msg::LowCmd::_motor_cmd_type arg)
  {
    msg_.motor_cmd = std::move(arg);
    return Init_LowCmd_reserve(msg_);
  }

private:
  ::unitree_hg::msg::LowCmd msg_;
};

class Init_LowCmd_mode_machine
{
public:
  explicit Init_LowCmd_mode_machine(::unitree_hg::msg::LowCmd & msg)
  : msg_(msg)
  {}
  Init_LowCmd_motor_cmd mode_machine(::unitree_hg::msg::LowCmd::_mode_machine_type arg)
  {
    msg_.mode_machine = std::move(arg);
    return Init_LowCmd_motor_cmd(msg_);
  }

private:
  ::unitree_hg::msg::LowCmd msg_;
};

class Init_LowCmd_mode_pr
{
public:
  Init_LowCmd_mode_pr()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LowCmd_mode_machine mode_pr(::unitree_hg::msg::LowCmd::_mode_pr_type arg)
  {
    msg_.mode_pr = std::move(arg);
    return Init_LowCmd_mode_machine(msg_);
  }

private:
  ::unitree_hg::msg::LowCmd msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::unitree_hg::msg::LowCmd>()
{
  return unitree_hg::msg::builder::Init_LowCmd_mode_pr();
}

}  // namespace unitree_hg

#endif  // UNITREE_HG__MSG__DETAIL__LOW_CMD__BUILDER_HPP_
