// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sd_msgs:msg/SDControl.idl
// generated code does not contain a copyright notice

#ifndef SD_MSGS__MSG__DETAIL__SD_CONTROL__STRUCT_HPP_
#define SD_MSGS__MSG__DETAIL__SD_CONTROL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__sd_msgs__msg__SDControl __attribute__((deprecated))
#else
# define DEPRECATED__sd_msgs__msg__SDControl __declspec(deprecated)
#endif

namespace sd_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SDControl_
{
  using Type = SDControl_<ContainerAllocator>;

  explicit SDControl_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->torque = 0.0;
      this->steer = 0.0;
    }
  }

  explicit SDControl_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->torque = 0.0;
      this->steer = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _torque_type =
    double;
  _torque_type torque;
  using _steer_type =
    double;
  _steer_type steer;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__torque(
    const double & _arg)
  {
    this->torque = _arg;
    return *this;
  }
  Type & set__steer(
    const double & _arg)
  {
    this->steer = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sd_msgs::msg::SDControl_<ContainerAllocator> *;
  using ConstRawPtr =
    const sd_msgs::msg::SDControl_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sd_msgs::msg::SDControl_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sd_msgs::msg::SDControl_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sd_msgs::msg::SDControl_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sd_msgs::msg::SDControl_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sd_msgs::msg::SDControl_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sd_msgs::msg::SDControl_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sd_msgs::msg::SDControl_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sd_msgs::msg::SDControl_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sd_msgs__msg__SDControl
    std::shared_ptr<sd_msgs::msg::SDControl_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sd_msgs__msg__SDControl
    std::shared_ptr<sd_msgs::msg::SDControl_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SDControl_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->torque != other.torque) {
      return false;
    }
    if (this->steer != other.steer) {
      return false;
    }
    return true;
  }
  bool operator!=(const SDControl_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SDControl_

// alias to use template instance with default allocator
using SDControl =
  sd_msgs::msg::SDControl_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace sd_msgs

#endif  // SD_MSGS__MSG__DETAIL__SD_CONTROL__STRUCT_HPP_
