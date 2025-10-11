// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from unitree_hg:msg/HandState.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_HG__MSG__DETAIL__HAND_STATE__STRUCT_HPP_
#define UNITREE_HG__MSG__DETAIL__HAND_STATE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'motor_state'
#include "unitree_hg/msg/detail/motor_state__struct.hpp"
// Member 'imu_state'
#include "unitree_hg/msg/detail/imu_state__struct.hpp"
// Member 'press_sensor_state'
#include "unitree_hg/msg/detail/press_sensor_state__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__unitree_hg__msg__HandState __attribute__((deprecated))
#else
# define DEPRECATED__unitree_hg__msg__HandState __declspec(deprecated)
#endif

namespace unitree_hg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct HandState_
{
  using Type = HandState_<ContainerAllocator>;

  explicit HandState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : imu_state(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->power_v = 0.0f;
      this->power_a = 0.0f;
      std::fill<typename std::array<uint32_t, 2>::iterator, uint32_t>(this->reserve.begin(), this->reserve.end(), 0ul);
    }
  }

  explicit HandState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : imu_state(_alloc, _init),
    reserve(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->power_v = 0.0f;
      this->power_a = 0.0f;
      std::fill<typename std::array<uint32_t, 2>::iterator, uint32_t>(this->reserve.begin(), this->reserve.end(), 0ul);
    }
  }

  // field types and members
  using _motor_state_type =
    std::vector<unitree_hg::msg::MotorState_<ContainerAllocator>, typename ContainerAllocator::template rebind<unitree_hg::msg::MotorState_<ContainerAllocator>>::other>;
  _motor_state_type motor_state;
  using _imu_state_type =
    unitree_hg::msg::IMUState_<ContainerAllocator>;
  _imu_state_type imu_state;
  using _press_sensor_state_type =
    std::vector<unitree_hg::msg::PressSensorState_<ContainerAllocator>, typename ContainerAllocator::template rebind<unitree_hg::msg::PressSensorState_<ContainerAllocator>>::other>;
  _press_sensor_state_type press_sensor_state;
  using _power_v_type =
    float;
  _power_v_type power_v;
  using _power_a_type =
    float;
  _power_a_type power_a;
  using _reserve_type =
    std::array<uint32_t, 2>;
  _reserve_type reserve;

  // setters for named parameter idiom
  Type & set__motor_state(
    const std::vector<unitree_hg::msg::MotorState_<ContainerAllocator>, typename ContainerAllocator::template rebind<unitree_hg::msg::MotorState_<ContainerAllocator>>::other> & _arg)
  {
    this->motor_state = _arg;
    return *this;
  }
  Type & set__imu_state(
    const unitree_hg::msg::IMUState_<ContainerAllocator> & _arg)
  {
    this->imu_state = _arg;
    return *this;
  }
  Type & set__press_sensor_state(
    const std::vector<unitree_hg::msg::PressSensorState_<ContainerAllocator>, typename ContainerAllocator::template rebind<unitree_hg::msg::PressSensorState_<ContainerAllocator>>::other> & _arg)
  {
    this->press_sensor_state = _arg;
    return *this;
  }
  Type & set__power_v(
    const float & _arg)
  {
    this->power_v = _arg;
    return *this;
  }
  Type & set__power_a(
    const float & _arg)
  {
    this->power_a = _arg;
    return *this;
  }
  Type & set__reserve(
    const std::array<uint32_t, 2> & _arg)
  {
    this->reserve = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    unitree_hg::msg::HandState_<ContainerAllocator> *;
  using ConstRawPtr =
    const unitree_hg::msg::HandState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<unitree_hg::msg::HandState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<unitree_hg::msg::HandState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      unitree_hg::msg::HandState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<unitree_hg::msg::HandState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      unitree_hg::msg::HandState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<unitree_hg::msg::HandState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<unitree_hg::msg::HandState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<unitree_hg::msg::HandState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__unitree_hg__msg__HandState
    std::shared_ptr<unitree_hg::msg::HandState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__unitree_hg__msg__HandState
    std::shared_ptr<unitree_hg::msg::HandState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const HandState_ & other) const
  {
    if (this->motor_state != other.motor_state) {
      return false;
    }
    if (this->imu_state != other.imu_state) {
      return false;
    }
    if (this->press_sensor_state != other.press_sensor_state) {
      return false;
    }
    if (this->power_v != other.power_v) {
      return false;
    }
    if (this->power_a != other.power_a) {
      return false;
    }
    if (this->reserve != other.reserve) {
      return false;
    }
    return true;
  }
  bool operator!=(const HandState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct HandState_

// alias to use template instance with default allocator
using HandState =
  unitree_hg::msg::HandState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace unitree_hg

#endif  // UNITREE_HG__MSG__DETAIL__HAND_STATE__STRUCT_HPP_
