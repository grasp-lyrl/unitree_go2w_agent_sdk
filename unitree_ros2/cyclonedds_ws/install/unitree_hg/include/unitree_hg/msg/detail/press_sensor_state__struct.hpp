// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from unitree_hg:msg/PressSensorState.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_HG__MSG__DETAIL__PRESS_SENSOR_STATE__STRUCT_HPP_
#define UNITREE_HG__MSG__DETAIL__PRESS_SENSOR_STATE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__unitree_hg__msg__PressSensorState __attribute__((deprecated))
#else
# define DEPRECATED__unitree_hg__msg__PressSensorState __declspec(deprecated)
#endif

namespace unitree_hg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PressSensorState_
{
  using Type = PressSensorState_<ContainerAllocator>;

  explicit PressSensorState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 12>::iterator, float>(this->pressure.begin(), this->pressure.end(), 0.0f);
      std::fill<typename std::array<float, 12>::iterator, float>(this->temperature.begin(), this->temperature.end(), 0.0f);
    }
  }

  explicit PressSensorState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pressure(_alloc),
    temperature(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 12>::iterator, float>(this->pressure.begin(), this->pressure.end(), 0.0f);
      std::fill<typename std::array<float, 12>::iterator, float>(this->temperature.begin(), this->temperature.end(), 0.0f);
    }
  }

  // field types and members
  using _pressure_type =
    std::array<float, 12>;
  _pressure_type pressure;
  using _temperature_type =
    std::array<float, 12>;
  _temperature_type temperature;

  // setters for named parameter idiom
  Type & set__pressure(
    const std::array<float, 12> & _arg)
  {
    this->pressure = _arg;
    return *this;
  }
  Type & set__temperature(
    const std::array<float, 12> & _arg)
  {
    this->temperature = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    unitree_hg::msg::PressSensorState_<ContainerAllocator> *;
  using ConstRawPtr =
    const unitree_hg::msg::PressSensorState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<unitree_hg::msg::PressSensorState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<unitree_hg::msg::PressSensorState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      unitree_hg::msg::PressSensorState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<unitree_hg::msg::PressSensorState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      unitree_hg::msg::PressSensorState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<unitree_hg::msg::PressSensorState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<unitree_hg::msg::PressSensorState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<unitree_hg::msg::PressSensorState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__unitree_hg__msg__PressSensorState
    std::shared_ptr<unitree_hg::msg::PressSensorState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__unitree_hg__msg__PressSensorState
    std::shared_ptr<unitree_hg::msg::PressSensorState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PressSensorState_ & other) const
  {
    if (this->pressure != other.pressure) {
      return false;
    }
    if (this->temperature != other.temperature) {
      return false;
    }
    return true;
  }
  bool operator!=(const PressSensorState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PressSensorState_

// alias to use template instance with default allocator
using PressSensorState =
  unitree_hg::msg::PressSensorState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace unitree_hg

#endif  // UNITREE_HG__MSG__DETAIL__PRESS_SENSOR_STATE__STRUCT_HPP_
