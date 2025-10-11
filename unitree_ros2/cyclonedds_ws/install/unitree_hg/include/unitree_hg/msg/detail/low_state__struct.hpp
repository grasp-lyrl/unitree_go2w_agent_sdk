// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from unitree_hg:msg/LowState.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_HG__MSG__DETAIL__LOW_STATE__STRUCT_HPP_
#define UNITREE_HG__MSG__DETAIL__LOW_STATE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'imu_state'
#include "unitree_hg/msg/detail/imu_state__struct.hpp"
// Member 'motor_state'
#include "unitree_hg/msg/detail/motor_state__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__unitree_hg__msg__LowState __attribute__((deprecated))
#else
# define DEPRECATED__unitree_hg__msg__LowState __declspec(deprecated)
#endif

namespace unitree_hg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LowState_
{
  using Type = LowState_<ContainerAllocator>;

  explicit LowState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : imu_state(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<uint32_t, 2>::iterator, uint32_t>(this->version.begin(), this->version.end(), 0ul);
      this->mode_pr = 0;
      this->mode_machine = 0;
      this->tick = 0ul;
      this->motor_state.fill(unitree_hg::msg::MotorState_<ContainerAllocator>{_init});
      std::fill<typename std::array<uint8_t, 40>::iterator, uint8_t>(this->wireless_remote.begin(), this->wireless_remote.end(), 0);
      std::fill<typename std::array<uint32_t, 4>::iterator, uint32_t>(this->reserve.begin(), this->reserve.end(), 0ul);
      this->crc = 0ul;
    }
  }

  explicit LowState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : version(_alloc),
    imu_state(_alloc, _init),
    motor_state(_alloc),
    wireless_remote(_alloc),
    reserve(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<uint32_t, 2>::iterator, uint32_t>(this->version.begin(), this->version.end(), 0ul);
      this->mode_pr = 0;
      this->mode_machine = 0;
      this->tick = 0ul;
      this->motor_state.fill(unitree_hg::msg::MotorState_<ContainerAllocator>{_alloc, _init});
      std::fill<typename std::array<uint8_t, 40>::iterator, uint8_t>(this->wireless_remote.begin(), this->wireless_remote.end(), 0);
      std::fill<typename std::array<uint32_t, 4>::iterator, uint32_t>(this->reserve.begin(), this->reserve.end(), 0ul);
      this->crc = 0ul;
    }
  }

  // field types and members
  using _version_type =
    std::array<uint32_t, 2>;
  _version_type version;
  using _mode_pr_type =
    uint8_t;
  _mode_pr_type mode_pr;
  using _mode_machine_type =
    uint8_t;
  _mode_machine_type mode_machine;
  using _tick_type =
    uint32_t;
  _tick_type tick;
  using _imu_state_type =
    unitree_hg::msg::IMUState_<ContainerAllocator>;
  _imu_state_type imu_state;
  using _motor_state_type =
    std::array<unitree_hg::msg::MotorState_<ContainerAllocator>, 35>;
  _motor_state_type motor_state;
  using _wireless_remote_type =
    std::array<uint8_t, 40>;
  _wireless_remote_type wireless_remote;
  using _reserve_type =
    std::array<uint32_t, 4>;
  _reserve_type reserve;
  using _crc_type =
    uint32_t;
  _crc_type crc;

  // setters for named parameter idiom
  Type & set__version(
    const std::array<uint32_t, 2> & _arg)
  {
    this->version = _arg;
    return *this;
  }
  Type & set__mode_pr(
    const uint8_t & _arg)
  {
    this->mode_pr = _arg;
    return *this;
  }
  Type & set__mode_machine(
    const uint8_t & _arg)
  {
    this->mode_machine = _arg;
    return *this;
  }
  Type & set__tick(
    const uint32_t & _arg)
  {
    this->tick = _arg;
    return *this;
  }
  Type & set__imu_state(
    const unitree_hg::msg::IMUState_<ContainerAllocator> & _arg)
  {
    this->imu_state = _arg;
    return *this;
  }
  Type & set__motor_state(
    const std::array<unitree_hg::msg::MotorState_<ContainerAllocator>, 35> & _arg)
  {
    this->motor_state = _arg;
    return *this;
  }
  Type & set__wireless_remote(
    const std::array<uint8_t, 40> & _arg)
  {
    this->wireless_remote = _arg;
    return *this;
  }
  Type & set__reserve(
    const std::array<uint32_t, 4> & _arg)
  {
    this->reserve = _arg;
    return *this;
  }
  Type & set__crc(
    const uint32_t & _arg)
  {
    this->crc = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    unitree_hg::msg::LowState_<ContainerAllocator> *;
  using ConstRawPtr =
    const unitree_hg::msg::LowState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<unitree_hg::msg::LowState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<unitree_hg::msg::LowState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      unitree_hg::msg::LowState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<unitree_hg::msg::LowState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      unitree_hg::msg::LowState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<unitree_hg::msg::LowState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<unitree_hg::msg::LowState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<unitree_hg::msg::LowState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__unitree_hg__msg__LowState
    std::shared_ptr<unitree_hg::msg::LowState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__unitree_hg__msg__LowState
    std::shared_ptr<unitree_hg::msg::LowState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LowState_ & other) const
  {
    if (this->version != other.version) {
      return false;
    }
    if (this->mode_pr != other.mode_pr) {
      return false;
    }
    if (this->mode_machine != other.mode_machine) {
      return false;
    }
    if (this->tick != other.tick) {
      return false;
    }
    if (this->imu_state != other.imu_state) {
      return false;
    }
    if (this->motor_state != other.motor_state) {
      return false;
    }
    if (this->wireless_remote != other.wireless_remote) {
      return false;
    }
    if (this->reserve != other.reserve) {
      return false;
    }
    if (this->crc != other.crc) {
      return false;
    }
    return true;
  }
  bool operator!=(const LowState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LowState_

// alias to use template instance with default allocator
using LowState =
  unitree_hg::msg::LowState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace unitree_hg

#endif  // UNITREE_HG__MSG__DETAIL__LOW_STATE__STRUCT_HPP_
