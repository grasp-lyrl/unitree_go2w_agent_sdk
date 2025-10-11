// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from unitree_go:msg/MotorCmds.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO__MSG__DETAIL__MOTOR_CMDS__STRUCT_HPP_
#define UNITREE_GO__MSG__DETAIL__MOTOR_CMDS__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'cmds'
#include "unitree_go/msg/detail/motor_cmd__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__unitree_go__msg__MotorCmds __attribute__((deprecated))
#else
# define DEPRECATED__unitree_go__msg__MotorCmds __declspec(deprecated)
#endif

namespace unitree_go
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MotorCmds_
{
  using Type = MotorCmds_<ContainerAllocator>;

  explicit MotorCmds_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit MotorCmds_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _cmds_type =
    std::vector<unitree_go::msg::MotorCmd_<ContainerAllocator>, typename ContainerAllocator::template rebind<unitree_go::msg::MotorCmd_<ContainerAllocator>>::other>;
  _cmds_type cmds;

  // setters for named parameter idiom
  Type & set__cmds(
    const std::vector<unitree_go::msg::MotorCmd_<ContainerAllocator>, typename ContainerAllocator::template rebind<unitree_go::msg::MotorCmd_<ContainerAllocator>>::other> & _arg)
  {
    this->cmds = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    unitree_go::msg::MotorCmds_<ContainerAllocator> *;
  using ConstRawPtr =
    const unitree_go::msg::MotorCmds_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<unitree_go::msg::MotorCmds_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<unitree_go::msg::MotorCmds_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      unitree_go::msg::MotorCmds_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<unitree_go::msg::MotorCmds_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      unitree_go::msg::MotorCmds_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<unitree_go::msg::MotorCmds_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<unitree_go::msg::MotorCmds_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<unitree_go::msg::MotorCmds_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__unitree_go__msg__MotorCmds
    std::shared_ptr<unitree_go::msg::MotorCmds_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__unitree_go__msg__MotorCmds
    std::shared_ptr<unitree_go::msg::MotorCmds_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotorCmds_ & other) const
  {
    if (this->cmds != other.cmds) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotorCmds_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotorCmds_

// alias to use template instance with default allocator
using MotorCmds =
  unitree_go::msg::MotorCmds_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace unitree_go

#endif  // UNITREE_GO__MSG__DETAIL__MOTOR_CMDS__STRUCT_HPP_
