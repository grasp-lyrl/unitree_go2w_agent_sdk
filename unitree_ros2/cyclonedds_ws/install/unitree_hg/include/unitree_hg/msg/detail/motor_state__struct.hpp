// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from unitree_hg:msg/MotorState.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_HG__MSG__DETAIL__MOTOR_STATE__STRUCT_HPP_
#define UNITREE_HG__MSG__DETAIL__MOTOR_STATE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__unitree_hg__msg__MotorState __attribute__((deprecated))
#else
# define DEPRECATED__unitree_hg__msg__MotorState __declspec(deprecated)
#endif

namespace unitree_hg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MotorState_
{
  using Type = MotorState_<ContainerAllocator>;

  explicit MotorState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode = 0;
      this->q = 0.0f;
      this->dq = 0.0f;
      this->ddq = 0.0f;
      this->tau_est = 0.0f;
      std::fill<typename std::array<int16_t, 2>::iterator, int16_t>(this->temperature.begin(), this->temperature.end(), 0);
      this->vol = 0.0f;
      std::fill<typename std::array<uint32_t, 2>::iterator, uint32_t>(this->sensor.begin(), this->sensor.end(), 0ul);
      this->motorstate = 0ul;
      std::fill<typename std::array<uint32_t, 4>::iterator, uint32_t>(this->reserve.begin(), this->reserve.end(), 0ul);
    }
  }

  explicit MotorState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : temperature(_alloc),
    sensor(_alloc),
    reserve(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode = 0;
      this->q = 0.0f;
      this->dq = 0.0f;
      this->ddq = 0.0f;
      this->tau_est = 0.0f;
      std::fill<typename std::array<int16_t, 2>::iterator, int16_t>(this->temperature.begin(), this->temperature.end(), 0);
      this->vol = 0.0f;
      std::fill<typename std::array<uint32_t, 2>::iterator, uint32_t>(this->sensor.begin(), this->sensor.end(), 0ul);
      this->motorstate = 0ul;
      std::fill<typename std::array<uint32_t, 4>::iterator, uint32_t>(this->reserve.begin(), this->reserve.end(), 0ul);
    }
  }

  // field types and members
  using _mode_type =
    uint8_t;
  _mode_type mode;
  using _q_type =
    float;
  _q_type q;
  using _dq_type =
    float;
  _dq_type dq;
  using _ddq_type =
    float;
  _ddq_type ddq;
  using _tau_est_type =
    float;
  _tau_est_type tau_est;
  using _temperature_type =
    std::array<int16_t, 2>;
  _temperature_type temperature;
  using _vol_type =
    float;
  _vol_type vol;
  using _sensor_type =
    std::array<uint32_t, 2>;
  _sensor_type sensor;
  using _motorstate_type =
    uint32_t;
  _motorstate_type motorstate;
  using _reserve_type =
    std::array<uint32_t, 4>;
  _reserve_type reserve;

  // setters for named parameter idiom
  Type & set__mode(
    const uint8_t & _arg)
  {
    this->mode = _arg;
    return *this;
  }
  Type & set__q(
    const float & _arg)
  {
    this->q = _arg;
    return *this;
  }
  Type & set__dq(
    const float & _arg)
  {
    this->dq = _arg;
    return *this;
  }
  Type & set__ddq(
    const float & _arg)
  {
    this->ddq = _arg;
    return *this;
  }
  Type & set__tau_est(
    const float & _arg)
  {
    this->tau_est = _arg;
    return *this;
  }
  Type & set__temperature(
    const std::array<int16_t, 2> & _arg)
  {
    this->temperature = _arg;
    return *this;
  }
  Type & set__vol(
    const float & _arg)
  {
    this->vol = _arg;
    return *this;
  }
  Type & set__sensor(
    const std::array<uint32_t, 2> & _arg)
  {
    this->sensor = _arg;
    return *this;
  }
  Type & set__motorstate(
    const uint32_t & _arg)
  {
    this->motorstate = _arg;
    return *this;
  }
  Type & set__reserve(
    const std::array<uint32_t, 4> & _arg)
  {
    this->reserve = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    unitree_hg::msg::MotorState_<ContainerAllocator> *;
  using ConstRawPtr =
    const unitree_hg::msg::MotorState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<unitree_hg::msg::MotorState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<unitree_hg::msg::MotorState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      unitree_hg::msg::MotorState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<unitree_hg::msg::MotorState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      unitree_hg::msg::MotorState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<unitree_hg::msg::MotorState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<unitree_hg::msg::MotorState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<unitree_hg::msg::MotorState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__unitree_hg__msg__MotorState
    std::shared_ptr<unitree_hg::msg::MotorState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__unitree_hg__msg__MotorState
    std::shared_ptr<unitree_hg::msg::MotorState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotorState_ & other) const
  {
    if (this->mode != other.mode) {
      return false;
    }
    if (this->q != other.q) {
      return false;
    }
    if (this->dq != other.dq) {
      return false;
    }
    if (this->ddq != other.ddq) {
      return false;
    }
    if (this->tau_est != other.tau_est) {
      return false;
    }
    if (this->temperature != other.temperature) {
      return false;
    }
    if (this->vol != other.vol) {
      return false;
    }
    if (this->sensor != other.sensor) {
      return false;
    }
    if (this->motorstate != other.motorstate) {
      return false;
    }
    if (this->reserve != other.reserve) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotorState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotorState_

// alias to use template instance with default allocator
using MotorState =
  unitree_hg::msg::MotorState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace unitree_hg

#endif  // UNITREE_HG__MSG__DETAIL__MOTOR_STATE__STRUCT_HPP_
