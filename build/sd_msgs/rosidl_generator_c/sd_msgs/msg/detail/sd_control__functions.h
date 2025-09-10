// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from sd_msgs:msg/SDControl.idl
// generated code does not contain a copyright notice

#ifndef SD_MSGS__MSG__DETAIL__SD_CONTROL__FUNCTIONS_H_
#define SD_MSGS__MSG__DETAIL__SD_CONTROL__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "sd_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "sd_msgs/msg/detail/sd_control__struct.h"

/// Initialize msg/SDControl message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * sd_msgs__msg__SDControl
 * )) before or use
 * sd_msgs__msg__SDControl__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_sd_msgs
bool
sd_msgs__msg__SDControl__init(sd_msgs__msg__SDControl * msg);

/// Finalize msg/SDControl message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_sd_msgs
void
sd_msgs__msg__SDControl__fini(sd_msgs__msg__SDControl * msg);

/// Create msg/SDControl message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * sd_msgs__msg__SDControl__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_sd_msgs
sd_msgs__msg__SDControl *
sd_msgs__msg__SDControl__create();

/// Destroy msg/SDControl message.
/**
 * It calls
 * sd_msgs__msg__SDControl__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_sd_msgs
void
sd_msgs__msg__SDControl__destroy(sd_msgs__msg__SDControl * msg);

/// Check for msg/SDControl message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_sd_msgs
bool
sd_msgs__msg__SDControl__are_equal(const sd_msgs__msg__SDControl * lhs, const sd_msgs__msg__SDControl * rhs);

/// Copy a msg/SDControl message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_sd_msgs
bool
sd_msgs__msg__SDControl__copy(
  const sd_msgs__msg__SDControl * input,
  sd_msgs__msg__SDControl * output);

/// Initialize array of msg/SDControl messages.
/**
 * It allocates the memory for the number of elements and calls
 * sd_msgs__msg__SDControl__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_sd_msgs
bool
sd_msgs__msg__SDControl__Sequence__init(sd_msgs__msg__SDControl__Sequence * array, size_t size);

/// Finalize array of msg/SDControl messages.
/**
 * It calls
 * sd_msgs__msg__SDControl__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_sd_msgs
void
sd_msgs__msg__SDControl__Sequence__fini(sd_msgs__msg__SDControl__Sequence * array);

/// Create array of msg/SDControl messages.
/**
 * It allocates the memory for the array and calls
 * sd_msgs__msg__SDControl__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_sd_msgs
sd_msgs__msg__SDControl__Sequence *
sd_msgs__msg__SDControl__Sequence__create(size_t size);

/// Destroy array of msg/SDControl messages.
/**
 * It calls
 * sd_msgs__msg__SDControl__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_sd_msgs
void
sd_msgs__msg__SDControl__Sequence__destroy(sd_msgs__msg__SDControl__Sequence * array);

/// Check for msg/SDControl message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_sd_msgs
bool
sd_msgs__msg__SDControl__Sequence__are_equal(const sd_msgs__msg__SDControl__Sequence * lhs, const sd_msgs__msg__SDControl__Sequence * rhs);

/// Copy an array of msg/SDControl messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_sd_msgs
bool
sd_msgs__msg__SDControl__Sequence__copy(
  const sd_msgs__msg__SDControl__Sequence * input,
  sd_msgs__msg__SDControl__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // SD_MSGS__MSG__DETAIL__SD_CONTROL__FUNCTIONS_H_
