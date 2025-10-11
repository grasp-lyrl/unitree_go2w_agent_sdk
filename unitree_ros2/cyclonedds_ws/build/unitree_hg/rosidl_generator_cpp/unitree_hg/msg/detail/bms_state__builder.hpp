// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from unitree_hg:msg/BmsState.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_HG__MSG__DETAIL__BMS_STATE__BUILDER_HPP_
#define UNITREE_HG__MSG__DETAIL__BMS_STATE__BUILDER_HPP_

#include "unitree_hg/msg/detail/bms_state__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace unitree_hg
{

namespace msg
{

namespace builder
{

class Init_BmsState_reserve
{
public:
  explicit Init_BmsState_reserve(::unitree_hg::msg::BmsState & msg)
  : msg_(msg)
  {}
  ::unitree_hg::msg::BmsState reserve(::unitree_hg::msg::BmsState::_reserve_type arg)
  {
    msg_.reserve = std::move(arg);
    return std::move(msg_);
  }

private:
  ::unitree_hg::msg::BmsState msg_;
};

class Init_BmsState_bmsstate
{
public:
  explicit Init_BmsState_bmsstate(::unitree_hg::msg::BmsState & msg)
  : msg_(msg)
  {}
  Init_BmsState_reserve bmsstate(::unitree_hg::msg::BmsState::_bmsstate_type arg)
  {
    msg_.bmsstate = std::move(arg);
    return Init_BmsState_reserve(msg_);
  }

private:
  ::unitree_hg::msg::BmsState msg_;
};

class Init_BmsState_manufacturer_date
{
public:
  explicit Init_BmsState_manufacturer_date(::unitree_hg::msg::BmsState & msg)
  : msg_(msg)
  {}
  Init_BmsState_bmsstate manufacturer_date(::unitree_hg::msg::BmsState::_manufacturer_date_type arg)
  {
    msg_.manufacturer_date = std::move(arg);
    return Init_BmsState_bmsstate(msg_);
  }

private:
  ::unitree_hg::msg::BmsState msg_;
};

class Init_BmsState_cycle
{
public:
  explicit Init_BmsState_cycle(::unitree_hg::msg::BmsState & msg)
  : msg_(msg)
  {}
  Init_BmsState_manufacturer_date cycle(::unitree_hg::msg::BmsState::_cycle_type arg)
  {
    msg_.cycle = std::move(arg);
    return Init_BmsState_manufacturer_date(msg_);
  }

private:
  ::unitree_hg::msg::BmsState msg_;
};

class Init_BmsState_temperature
{
public:
  explicit Init_BmsState_temperature(::unitree_hg::msg::BmsState & msg)
  : msg_(msg)
  {}
  Init_BmsState_cycle temperature(::unitree_hg::msg::BmsState::_temperature_type arg)
  {
    msg_.temperature = std::move(arg);
    return Init_BmsState_cycle(msg_);
  }

private:
  ::unitree_hg::msg::BmsState msg_;
};

class Init_BmsState_soh
{
public:
  explicit Init_BmsState_soh(::unitree_hg::msg::BmsState & msg)
  : msg_(msg)
  {}
  Init_BmsState_temperature soh(::unitree_hg::msg::BmsState::_soh_type arg)
  {
    msg_.soh = std::move(arg);
    return Init_BmsState_temperature(msg_);
  }

private:
  ::unitree_hg::msg::BmsState msg_;
};

class Init_BmsState_soc
{
public:
  explicit Init_BmsState_soc(::unitree_hg::msg::BmsState & msg)
  : msg_(msg)
  {}
  Init_BmsState_soh soc(::unitree_hg::msg::BmsState::_soc_type arg)
  {
    msg_.soc = std::move(arg);
    return Init_BmsState_soh(msg_);
  }

private:
  ::unitree_hg::msg::BmsState msg_;
};

class Init_BmsState_current
{
public:
  explicit Init_BmsState_current(::unitree_hg::msg::BmsState & msg)
  : msg_(msg)
  {}
  Init_BmsState_soc current(::unitree_hg::msg::BmsState::_current_type arg)
  {
    msg_.current = std::move(arg);
    return Init_BmsState_soc(msg_);
  }

private:
  ::unitree_hg::msg::BmsState msg_;
};

class Init_BmsState_bmsvoltage
{
public:
  explicit Init_BmsState_bmsvoltage(::unitree_hg::msg::BmsState & msg)
  : msg_(msg)
  {}
  Init_BmsState_current bmsvoltage(::unitree_hg::msg::BmsState::_bmsvoltage_type arg)
  {
    msg_.bmsvoltage = std::move(arg);
    return Init_BmsState_current(msg_);
  }

private:
  ::unitree_hg::msg::BmsState msg_;
};

class Init_BmsState_cell_vol
{
public:
  explicit Init_BmsState_cell_vol(::unitree_hg::msg::BmsState & msg)
  : msg_(msg)
  {}
  Init_BmsState_bmsvoltage cell_vol(::unitree_hg::msg::BmsState::_cell_vol_type arg)
  {
    msg_.cell_vol = std::move(arg);
    return Init_BmsState_bmsvoltage(msg_);
  }

private:
  ::unitree_hg::msg::BmsState msg_;
};

class Init_BmsState_fn
{
public:
  explicit Init_BmsState_fn(::unitree_hg::msg::BmsState & msg)
  : msg_(msg)
  {}
  Init_BmsState_cell_vol fn(::unitree_hg::msg::BmsState::_fn_type arg)
  {
    msg_.fn = std::move(arg);
    return Init_BmsState_cell_vol(msg_);
  }

private:
  ::unitree_hg::msg::BmsState msg_;
};

class Init_BmsState_version_low
{
public:
  explicit Init_BmsState_version_low(::unitree_hg::msg::BmsState & msg)
  : msg_(msg)
  {}
  Init_BmsState_fn version_low(::unitree_hg::msg::BmsState::_version_low_type arg)
  {
    msg_.version_low = std::move(arg);
    return Init_BmsState_fn(msg_);
  }

private:
  ::unitree_hg::msg::BmsState msg_;
};

class Init_BmsState_version_high
{
public:
  Init_BmsState_version_high()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BmsState_version_low version_high(::unitree_hg::msg::BmsState::_version_high_type arg)
  {
    msg_.version_high = std::move(arg);
    return Init_BmsState_version_low(msg_);
  }

private:
  ::unitree_hg::msg::BmsState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::unitree_hg::msg::BmsState>()
{
  return unitree_hg::msg::builder::Init_BmsState_version_high();
}

}  // namespace unitree_hg

#endif  // UNITREE_HG__MSG__DETAIL__BMS_STATE__BUILDER_HPP_
