// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from unitree_hg:msg/BmsCmd.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_HG__MSG__DETAIL__BMS_CMD__BUILDER_HPP_
#define UNITREE_HG__MSG__DETAIL__BMS_CMD__BUILDER_HPP_

#include "unitree_hg/msg/detail/bms_cmd__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace unitree_hg
{

namespace msg
{

namespace builder
{

class Init_BmsCmd_reserve
{
public:
  explicit Init_BmsCmd_reserve(::unitree_hg::msg::BmsCmd & msg)
  : msg_(msg)
  {}
  ::unitree_hg::msg::BmsCmd reserve(::unitree_hg::msg::BmsCmd::_reserve_type arg)
  {
    msg_.reserve = std::move(arg);
    return std::move(msg_);
  }

private:
  ::unitree_hg::msg::BmsCmd msg_;
};

class Init_BmsCmd_cmd
{
public:
  Init_BmsCmd_cmd()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BmsCmd_reserve cmd(::unitree_hg::msg::BmsCmd::_cmd_type arg)
  {
    msg_.cmd = std::move(arg);
    return Init_BmsCmd_reserve(msg_);
  }

private:
  ::unitree_hg::msg::BmsCmd msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::unitree_hg::msg::BmsCmd>()
{
  return unitree_hg::msg::builder::Init_BmsCmd_cmd();
}

}  // namespace unitree_hg

#endif  // UNITREE_HG__MSG__DETAIL__BMS_CMD__BUILDER_HPP_
