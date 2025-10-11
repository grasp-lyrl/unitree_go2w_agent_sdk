// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from unitree_hg:msg/BmsState.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_HG__MSG__DETAIL__BMS_STATE__STRUCT_HPP_
#define UNITREE_HG__MSG__DETAIL__BMS_STATE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__unitree_hg__msg__BmsState __attribute__((deprecated))
#else
# define DEPRECATED__unitree_hg__msg__BmsState __declspec(deprecated)
#endif

namespace unitree_hg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BmsState_
{
  using Type = BmsState_<ContainerAllocator>;

  explicit BmsState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->version_high = 0;
      this->version_low = 0;
      this->fn = 0;
      std::fill<typename std::array<uint16_t, 40>::iterator, uint16_t>(this->cell_vol.begin(), this->cell_vol.end(), 0);
      std::fill<typename std::array<uint32_t, 3>::iterator, uint32_t>(this->bmsvoltage.begin(), this->bmsvoltage.end(), 0ul);
      this->current = 0l;
      this->soc = 0;
      this->soh = 0;
      std::fill<typename std::array<int16_t, 12>::iterator, int16_t>(this->temperature.begin(), this->temperature.end(), 0);
      this->cycle = 0;
      this->manufacturer_date = 0;
      std::fill<typename std::array<uint32_t, 5>::iterator, uint32_t>(this->bmsstate.begin(), this->bmsstate.end(), 0ul);
      std::fill<typename std::array<uint32_t, 3>::iterator, uint32_t>(this->reserve.begin(), this->reserve.end(), 0ul);
    }
  }

  explicit BmsState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : cell_vol(_alloc),
    bmsvoltage(_alloc),
    temperature(_alloc),
    bmsstate(_alloc),
    reserve(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->version_high = 0;
      this->version_low = 0;
      this->fn = 0;
      std::fill<typename std::array<uint16_t, 40>::iterator, uint16_t>(this->cell_vol.begin(), this->cell_vol.end(), 0);
      std::fill<typename std::array<uint32_t, 3>::iterator, uint32_t>(this->bmsvoltage.begin(), this->bmsvoltage.end(), 0ul);
      this->current = 0l;
      this->soc = 0;
      this->soh = 0;
      std::fill<typename std::array<int16_t, 12>::iterator, int16_t>(this->temperature.begin(), this->temperature.end(), 0);
      this->cycle = 0;
      this->manufacturer_date = 0;
      std::fill<typename std::array<uint32_t, 5>::iterator, uint32_t>(this->bmsstate.begin(), this->bmsstate.end(), 0ul);
      std::fill<typename std::array<uint32_t, 3>::iterator, uint32_t>(this->reserve.begin(), this->reserve.end(), 0ul);
    }
  }

  // field types and members
  using _version_high_type =
    uint8_t;
  _version_high_type version_high;
  using _version_low_type =
    uint8_t;
  _version_low_type version_low;
  using _fn_type =
    uint8_t;
  _fn_type fn;
  using _cell_vol_type =
    std::array<uint16_t, 40>;
  _cell_vol_type cell_vol;
  using _bmsvoltage_type =
    std::array<uint32_t, 3>;
  _bmsvoltage_type bmsvoltage;
  using _current_type =
    int32_t;
  _current_type current;
  using _soc_type =
    uint8_t;
  _soc_type soc;
  using _soh_type =
    uint8_t;
  _soh_type soh;
  using _temperature_type =
    std::array<int16_t, 12>;
  _temperature_type temperature;
  using _cycle_type =
    uint16_t;
  _cycle_type cycle;
  using _manufacturer_date_type =
    uint16_t;
  _manufacturer_date_type manufacturer_date;
  using _bmsstate_type =
    std::array<uint32_t, 5>;
  _bmsstate_type bmsstate;
  using _reserve_type =
    std::array<uint32_t, 3>;
  _reserve_type reserve;

  // setters for named parameter idiom
  Type & set__version_high(
    const uint8_t & _arg)
  {
    this->version_high = _arg;
    return *this;
  }
  Type & set__version_low(
    const uint8_t & _arg)
  {
    this->version_low = _arg;
    return *this;
  }
  Type & set__fn(
    const uint8_t & _arg)
  {
    this->fn = _arg;
    return *this;
  }
  Type & set__cell_vol(
    const std::array<uint16_t, 40> & _arg)
  {
    this->cell_vol = _arg;
    return *this;
  }
  Type & set__bmsvoltage(
    const std::array<uint32_t, 3> & _arg)
  {
    this->bmsvoltage = _arg;
    return *this;
  }
  Type & set__current(
    const int32_t & _arg)
  {
    this->current = _arg;
    return *this;
  }
  Type & set__soc(
    const uint8_t & _arg)
  {
    this->soc = _arg;
    return *this;
  }
  Type & set__soh(
    const uint8_t & _arg)
  {
    this->soh = _arg;
    return *this;
  }
  Type & set__temperature(
    const std::array<int16_t, 12> & _arg)
  {
    this->temperature = _arg;
    return *this;
  }
  Type & set__cycle(
    const uint16_t & _arg)
  {
    this->cycle = _arg;
    return *this;
  }
  Type & set__manufacturer_date(
    const uint16_t & _arg)
  {
    this->manufacturer_date = _arg;
    return *this;
  }
  Type & set__bmsstate(
    const std::array<uint32_t, 5> & _arg)
  {
    this->bmsstate = _arg;
    return *this;
  }
  Type & set__reserve(
    const std::array<uint32_t, 3> & _arg)
  {
    this->reserve = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    unitree_hg::msg::BmsState_<ContainerAllocator> *;
  using ConstRawPtr =
    const unitree_hg::msg::BmsState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<unitree_hg::msg::BmsState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<unitree_hg::msg::BmsState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      unitree_hg::msg::BmsState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<unitree_hg::msg::BmsState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      unitree_hg::msg::BmsState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<unitree_hg::msg::BmsState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<unitree_hg::msg::BmsState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<unitree_hg::msg::BmsState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__unitree_hg__msg__BmsState
    std::shared_ptr<unitree_hg::msg::BmsState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__unitree_hg__msg__BmsState
    std::shared_ptr<unitree_hg::msg::BmsState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BmsState_ & other) const
  {
    if (this->version_high != other.version_high) {
      return false;
    }
    if (this->version_low != other.version_low) {
      return false;
    }
    if (this->fn != other.fn) {
      return false;
    }
    if (this->cell_vol != other.cell_vol) {
      return false;
    }
    if (this->bmsvoltage != other.bmsvoltage) {
      return false;
    }
    if (this->current != other.current) {
      return false;
    }
    if (this->soc != other.soc) {
      return false;
    }
    if (this->soh != other.soh) {
      return false;
    }
    if (this->temperature != other.temperature) {
      return false;
    }
    if (this->cycle != other.cycle) {
      return false;
    }
    if (this->manufacturer_date != other.manufacturer_date) {
      return false;
    }
    if (this->bmsstate != other.bmsstate) {
      return false;
    }
    if (this->reserve != other.reserve) {
      return false;
    }
    return true;
  }
  bool operator!=(const BmsState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BmsState_

// alias to use template instance with default allocator
using BmsState =
  unitree_hg::msg::BmsState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace unitree_hg

#endif  // UNITREE_HG__MSG__DETAIL__BMS_STATE__STRUCT_HPP_
