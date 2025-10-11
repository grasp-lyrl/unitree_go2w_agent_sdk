// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from unitree_hg:msg/LowCmd.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_HG__MSG__DETAIL__LOW_CMD__STRUCT_HPP_
#define UNITREE_HG__MSG__DETAIL__LOW_CMD__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'motor_cmd'
#include "unitree_hg/msg/detail/motor_cmd__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__unitree_hg__msg__LowCmd __attribute__((deprecated))
#else
# define DEPRECATED__unitree_hg__msg__LowCmd __declspec(deprecated)
#endif

namespace unitree_hg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LowCmd_
{
  using Type = LowCmd_<ContainerAllocator>;

  explicit LowCmd_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode_pr = 0;
      this->mode_machine = 0;
      this->motor_cmd.fill(unitree_hg::msg::MotorCmd_<ContainerAllocator>{_init});
      std::fill<typename std::array<uint32_t, 4>::iterator, uint32_t>(this->reserve.begin(), this->reserve.end(), 0ul);
      this->crc = 0ul;
    }
  }

  explicit LowCmd_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : motor_cmd(_alloc),
    reserve(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode_pr = 0;
      this->mode_machine = 0;
      this->motor_cmd.fill(unitree_hg::msg::MotorCmd_<ContainerAllocator>{_alloc, _init});
      std::fill<typename std::array<uint32_t, 4>::iterator, uint32_t>(this->reserve.begin(), this->reserve.end(), 0ul);
      this->crc = 0ul;
    }
  }

  // field types and members
  using _mode_pr_type =
    uint8_t;
  _mode_pr_type mode_pr;
  using _mode_machine_type =
    uint8_t;
  _mode_machine_type mode_machine;
  using _motor_cmd_type =
    std::array<unitree_hg::msg::MotorCmd_<ContainerAllocator>, 35>;
  _motor_cmd_type motor_cmd;
  using _reserve_type =
    std::array<uint32_t, 4>;
  _reserve_type reserve;
  using _crc_type =
    uint32_t;
  _crc_type crc;

  // setters for named parameter idiom
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
  Type & set__motor_cmd(
    const std::array<unitree_hg::msg::MotorCmd_<ContainerAllocator>, 35> & _arg)
  {
    this->motor_cmd = _arg;
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
    unitree_hg::msg::LowCmd_<ContainerAllocator> *;
  using ConstRawPtr =
    const unitree_hg::msg::LowCmd_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<unitree_hg::msg::LowCmd_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<unitree_hg::msg::LowCmd_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      unitree_hg::msg::LowCmd_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<unitree_hg::msg::LowCmd_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      unitree_hg::msg::LowCmd_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<unitree_hg::msg::LowCmd_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<unitree_hg::msg::LowCmd_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<unitree_hg::msg::LowCmd_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__unitree_hg__msg__LowCmd
    std::shared_ptr<unitree_hg::msg::LowCmd_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__unitree_hg__msg__LowCmd
    std::shared_ptr<unitree_hg::msg::LowCmd_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LowCmd_ & other) const
  {
    if (this->mode_pr != other.mode_pr) {
      return false;
    }
    if (this->mode_machine != other.mode_machine) {
      return false;
    }
    if (this->motor_cmd != other.motor_cmd) {
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
  bool operator!=(const LowCmd_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LowCmd_

// alias to use template instance with default allocator
using LowCmd =
  unitree_hg::msg::LowCmd_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace unitree_hg

#endif  // UNITREE_HG__MSG__DETAIL__LOW_CMD__STRUCT_HPP_
