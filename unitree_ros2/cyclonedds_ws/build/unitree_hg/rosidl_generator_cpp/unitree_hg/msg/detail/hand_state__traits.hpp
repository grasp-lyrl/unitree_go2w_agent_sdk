// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from unitree_hg:msg/HandState.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_HG__MSG__DETAIL__HAND_STATE__TRAITS_HPP_
#define UNITREE_HG__MSG__DETAIL__HAND_STATE__TRAITS_HPP_

#include "unitree_hg/msg/detail/hand_state__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'imu_state'
#include "unitree_hg/msg/detail/imu_state__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<unitree_hg::msg::HandState>()
{
  return "unitree_hg::msg::HandState";
}

template<>
inline const char * name<unitree_hg::msg::HandState>()
{
  return "unitree_hg/msg/HandState";
}

template<>
struct has_fixed_size<unitree_hg::msg::HandState>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<unitree_hg::msg::HandState>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<unitree_hg::msg::HandState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UNITREE_HG__MSG__DETAIL__HAND_STATE__TRAITS_HPP_
