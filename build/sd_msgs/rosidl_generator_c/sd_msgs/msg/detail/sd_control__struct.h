// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sd_msgs:msg/SDControl.idl
// generated code does not contain a copyright notice

#ifndef SD_MSGS__MSG__DETAIL__SD_CONTROL__STRUCT_H_
#define SD_MSGS__MSG__DETAIL__SD_CONTROL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/SDControl in the package sd_msgs.
typedef struct sd_msgs__msg__SDControl
{
  std_msgs__msg__Header header;
  /// Range -100 to 100, -100 is max brake, +100 is max throttle
  double torque;
  /// Range -100 to +100, +100 is maximum left turn
  double steer;
} sd_msgs__msg__SDControl;

// Struct for a sequence of sd_msgs__msg__SDControl.
typedef struct sd_msgs__msg__SDControl__Sequence
{
  sd_msgs__msg__SDControl * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sd_msgs__msg__SDControl__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SD_MSGS__MSG__DETAIL__SD_CONTROL__STRUCT_H_
