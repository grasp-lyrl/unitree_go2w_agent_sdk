// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from unitree_hg:msg/HandCmd.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_HG__MSG__DETAIL__HAND_CMD__STRUCT_HPP_
#define UNITREE_HG__MSG__DETAIL__HAND_CMD__STRUCT_HPP_

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
# define DEPRECATED__unitree_hg__msg__HandCmd __attribute__((deprecated))
#else
# define DEPRECATED__unitree_hg__msg__HandCmd __declspec(deprecated)
#endif

namespace unitree_hg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct HandCmd_
{
  using Type = HandCmd_<ContainerAllocator>;

  explicit HandCmd_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit HandCmd_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _motor_cmd_type =
    std::vector<unitree_hg::msg::MotorCmd_<ContainerAllocator>, typename ContainerAllocator::template rebind<unitree_hg::msg::MotorCmd_<ContainerAllocator>>::other>;
  _motor_cmd_type motor_cmd;

  // setters for named parameter idiom
  Type & set__motor_cmd(
    const std::vector<unitree_hg::msg::MotorCmd_<ContainerAllocator>, typename ContainerAllocator::template rebind<unitree_hg::msg::MotorCmd_<ContainerAllocator>>::other> & _arg)
  {
    this->motor_cmd = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    unitree_hg::msg::HandCmd_<ContainerAllocator> *;
  using ConstRawPtr =
    const unitree_hg::msg::HandCmd_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<unitree_hg::msg::HandCmd_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<unitree_hg::msg::HandCmd_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      unitree_hg::msg::HandCmd_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<unitree_hg::msg::HandCmd_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      unitree_hg::msg::HandCmd_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<unitree_hg::msg::HandCmd_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<unitree_hg::msg::HandCmd_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<unitree_hg::msg::HandCmd_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__unitree_hg__msg__HandCmd
    std::shared_ptr<unitree_hg::msg::HandCmd_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__unitree_hg__msg__HandCmd
    std::shared_ptr<unitree_hg::msg::HandCmd_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const HandCmd_ & other) const
  {
    if (this->motor_cmd != other.motor_cmd) {
      return false;
    }
    return true;
  }
  bool operator!=(const HandCmd_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct HandCmd_

// alias to use template instance with default allocator
using HandCmd =
  unitree_hg::msg::HandCmd_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace unitree_hg

#endif  // UNITREE_HG__MSG__DETAIL__HAND_CMD__STRUCT_HPP_
