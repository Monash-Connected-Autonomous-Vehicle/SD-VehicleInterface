// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from sd_msgs:msg/SDControl.idl
// generated code does not contain a copyright notice

#ifndef SD_MSGS__MSG__DETAIL__SD_CONTROL__BUILDER_HPP_
#define SD_MSGS__MSG__DETAIL__SD_CONTROL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "sd_msgs/msg/detail/sd_control__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace sd_msgs
{

namespace msg
{

namespace builder
{

class Init_SDControl_steer
{
public:
  explicit Init_SDControl_steer(::sd_msgs::msg::SDControl & msg)
  : msg_(msg)
  {}
  ::sd_msgs::msg::SDControl steer(::sd_msgs::msg::SDControl::_steer_type arg)
  {
    msg_.steer = std::move(arg);
    return std::move(msg_);
  }

private:
  ::sd_msgs::msg::SDControl msg_;
};

class Init_SDControl_torque
{
public:
  explicit Init_SDControl_torque(::sd_msgs::msg::SDControl & msg)
  : msg_(msg)
  {}
  Init_SDControl_steer torque(::sd_msgs::msg::SDControl::_torque_type arg)
  {
    msg_.torque = std::move(arg);
    return Init_SDControl_steer(msg_);
  }

private:
  ::sd_msgs::msg::SDControl msg_;
};

class Init_SDControl_header
{
public:
  Init_SDControl_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SDControl_torque header(::sd_msgs::msg::SDControl::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_SDControl_torque(msg_);
  }

private:
  ::sd_msgs::msg::SDControl msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::sd_msgs::msg::SDControl>()
{
  return sd_msgs::msg::builder::Init_SDControl_header();
}

}  // namespace sd_msgs

#endif  // SD_MSGS__MSG__DETAIL__SD_CONTROL__BUILDER_HPP_
