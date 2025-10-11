// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from unitree_hg:msg/MotorCmd.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_HG__MSG__DETAIL__MOTOR_CMD__TRAITS_HPP_
#define UNITREE_HG__MSG__DETAIL__MOTOR_CMD__TRAITS_HPP_

#include "unitree_hg/msg/detail/motor_cmd__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<unitree_hg::msg::MotorCmd>()
{
  return "unitree_hg::msg::MotorCmd";
}

template<>
inline const char * name<unitree_hg::msg::MotorCmd>()
{
  return "unitree_hg/msg/MotorCmd";
}

template<>
struct has_fixed_size<unitree_hg::msg::MotorCmd>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<unitree_hg::msg::MotorCmd>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<unitree_hg::msg::MotorCmd>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UNITREE_HG__MSG__DETAIL__MOTOR_CMD__TRAITS_HPP_
