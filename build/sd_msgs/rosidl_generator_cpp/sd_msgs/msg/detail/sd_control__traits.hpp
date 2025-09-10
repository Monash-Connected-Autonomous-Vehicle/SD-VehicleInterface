// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from sd_msgs:msg/SDControl.idl
// generated code does not contain a copyright notice

#ifndef SD_MSGS__MSG__DETAIL__SD_CONTROL__TRAITS_HPP_
#define SD_MSGS__MSG__DETAIL__SD_CONTROL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "sd_msgs/msg/detail/sd_control__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace sd_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const SDControl & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: torque
  {
    out << "torque: ";
    rosidl_generator_traits::value_to_yaml(msg.torque, out);
    out << ", ";
  }

  // member: steer
  {
    out << "steer: ";
    rosidl_generator_traits::value_to_yaml(msg.steer, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SDControl & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: torque
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "torque: ";
    rosidl_generator_traits::value_to_yaml(msg.torque, out);
    out << "\n";
  }

  // member: steer
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "steer: ";
    rosidl_generator_traits::value_to_yaml(msg.steer, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SDControl & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace sd_msgs

namespace rosidl_generator_traits
{

[[deprecated("use sd_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const sd_msgs::msg::SDControl & msg,
  std::ostream & out, size_t indentation = 0)
{
  sd_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use sd_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const sd_msgs::msg::SDControl & msg)
{
  return sd_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<sd_msgs::msg::SDControl>()
{
  return "sd_msgs::msg::SDControl";
}

template<>
inline const char * name<sd_msgs::msg::SDControl>()
{
  return "sd_msgs/msg/SDControl";
}

template<>
struct has_fixed_size<sd_msgs::msg::SDControl>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<sd_msgs::msg::SDControl>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<sd_msgs::msg::SDControl>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SD_MSGS__MSG__DETAIL__SD_CONTROL__TRAITS_HPP_
