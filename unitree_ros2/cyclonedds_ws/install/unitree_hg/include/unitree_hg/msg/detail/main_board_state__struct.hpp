// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from unitree_hg:msg/MainBoardState.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_HG__MSG__DETAIL__MAIN_BOARD_STATE__STRUCT_HPP_
#define UNITREE_HG__MSG__DETAIL__MAIN_BOARD_STATE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__unitree_hg__msg__MainBoardState __attribute__((deprecated))
#else
# define DEPRECATED__unitree_hg__msg__MainBoardState __declspec(deprecated)
#endif

namespace unitree_hg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MainBoardState_
{
  using Type = MainBoardState_<ContainerAllocator>;

  explicit MainBoardState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<uint16_t, 6>::iterator, uint16_t>(this->fan_state.begin(), this->fan_state.end(), 0);
      std::fill<typename std::array<int16_t, 6>::iterator, int16_t>(this->temperature.begin(), this->temperature.end(), 0);
      std::fill<typename std::array<float, 6>::iterator, float>(this->value.begin(), this->value.end(), 0.0f);
      std::fill<typename std::array<uint32_t, 6>::iterator, uint32_t>(this->state.begin(), this->state.end(), 0ul);
    }
  }

  explicit MainBoardState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : fan_state(_alloc),
    temperature(_alloc),
    value(_alloc),
    state(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<uint16_t, 6>::iterator, uint16_t>(this->fan_state.begin(), this->fan_state.end(), 0);
      std::fill<typename std::array<int16_t, 6>::iterator, int16_t>(this->temperature.begin(), this->temperature.end(), 0);
      std::fill<typename std::array<float, 6>::iterator, float>(this->value.begin(), this->value.end(), 0.0f);
      std::fill<typename std::array<uint32_t, 6>::iterator, uint32_t>(this->state.begin(), this->state.end(), 0ul);
    }
  }

  // field types and members
  using _fan_state_type =
    std::array<uint16_t, 6>;
  _fan_state_type fan_state;
  using _temperature_type =
    std::array<int16_t, 6>;
  _temperature_type temperature;
  using _value_type =
    std::array<float, 6>;
  _value_type value;
  using _state_type =
    std::array<uint32_t, 6>;
  _state_type state;

  // setters for named parameter idiom
  Type & set__fan_state(
    const std::array<uint16_t, 6> & _arg)
  {
    this->fan_state = _arg;
    return *this;
  }
  Type & set__temperature(
    const std::array<int16_t, 6> & _arg)
  {
    this->temperature = _arg;
    return *this;
  }
  Type & set__value(
    const std::array<float, 6> & _arg)
  {
    this->value = _arg;
    return *this;
  }
  Type & set__state(
    const std::array<uint32_t, 6> & _arg)
  {
    this->state = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    unitree_hg::msg::MainBoardState_<ContainerAllocator> *;
  using ConstRawPtr =
    const unitree_hg::msg::MainBoardState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<unitree_hg::msg::MainBoardState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<unitree_hg::msg::MainBoardState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      unitree_hg::msg::MainBoardState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<unitree_hg::msg::MainBoardState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      unitree_hg::msg::MainBoardState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<unitree_hg::msg::MainBoardState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<unitree_hg::msg::MainBoardState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<unitree_hg::msg::MainBoardState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__unitree_hg__msg__MainBoardState
    std::shared_ptr<unitree_hg::msg::MainBoardState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__unitree_hg__msg__MainBoardState
    std::shared_ptr<unitree_hg::msg::MainBoardState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MainBoardState_ & other) const
  {
    if (this->fan_state != other.fan_state) {
      return false;
    }
    if (this->temperature != other.temperature) {
      return false;
    }
    if (this->value != other.value) {
      return false;
    }
    if (this->state != other.state) {
      return false;
    }
    return true;
  }
  bool operator!=(const MainBoardState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MainBoardState_

// alias to use template instance with default allocator
using MainBoardState =
  unitree_hg::msg::MainBoardState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace unitree_hg

#endif  // UNITREE_HG__MSG__DETAIL__MAIN_BOARD_STATE__STRUCT_HPP_
