// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from unitree_hg:msg/LowCmd.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_HG__MSG__DETAIL__LOW_CMD__TRAITS_HPP_
#define UNITREE_HG__MSG__DETAIL__LOW_CMD__TRAITS_HPP_

#include "unitree_hg/msg/detail/low_cmd__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'motor_cmd'
#include "unitree_hg/msg/detail/motor_cmd__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<unitree_hg::msg::LowCmd>()
{
  return "unitree_hg::msg::LowCmd";
}

template<>
inline const char * name<unitree_hg::msg::LowCmd>()
{
  return "unitree_hg/msg/LowCmd";
}

template<>
struct has_fixed_size<unitree_hg::msg::LowCmd>
  : std::integral_constant<bool, has_fixed_size<unitree_hg::msg::MotorCmd>::value> {};

template<>
struct has_bounded_size<unitree_hg::msg::LowCmd>
  : std::integral_constant<bool, has_bounded_size<unitree_hg::msg::MotorCmd>::value> {};

template<>
struct is_message<unitree_hg::msg::LowCmd>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UNITREE_HG__MSG__DETAIL__LOW_CMD__TRAITS_HPP_
