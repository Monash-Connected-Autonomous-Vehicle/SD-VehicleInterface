// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from sd_msgs:msg/SDControl.idl
// generated code does not contain a copyright notice
#include "sd_msgs/msg/detail/sd_control__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
sd_msgs__msg__SDControl__init(sd_msgs__msg__SDControl * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    sd_msgs__msg__SDControl__fini(msg);
    return false;
  }
  // torque
  // steer
  return true;
}

void
sd_msgs__msg__SDControl__fini(sd_msgs__msg__SDControl * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // torque
  // steer
}

bool
sd_msgs__msg__SDControl__are_equal(const sd_msgs__msg__SDControl * lhs, const sd_msgs__msg__SDControl * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // torque
  if (lhs->torque != rhs->torque) {
    return false;
  }
  // steer
  if (lhs->steer != rhs->steer) {
    return false;
  }
  return true;
}

bool
sd_msgs__msg__SDControl__copy(
  const sd_msgs__msg__SDControl * input,
  sd_msgs__msg__SDControl * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // torque
  output->torque = input->torque;
  // steer
  output->steer = input->steer;
  return true;
}

sd_msgs__msg__SDControl *
sd_msgs__msg__SDControl__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sd_msgs__msg__SDControl * msg = (sd_msgs__msg__SDControl *)allocator.allocate(sizeof(sd_msgs__msg__SDControl), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(sd_msgs__msg__SDControl));
  bool success = sd_msgs__msg__SDControl__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
sd_msgs__msg__SDControl__destroy(sd_msgs__msg__SDControl * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    sd_msgs__msg__SDControl__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
sd_msgs__msg__SDControl__Sequence__init(sd_msgs__msg__SDControl__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sd_msgs__msg__SDControl * data = NULL;

  if (size) {
    data = (sd_msgs__msg__SDControl *)allocator.zero_allocate(size, sizeof(sd_msgs__msg__SDControl), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = sd_msgs__msg__SDControl__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        sd_msgs__msg__SDControl__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
sd_msgs__msg__SDControl__Sequence__fini(sd_msgs__msg__SDControl__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      sd_msgs__msg__SDControl__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

sd_msgs__msg__SDControl__Sequence *
sd_msgs__msg__SDControl__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sd_msgs__msg__SDControl__Sequence * array = (sd_msgs__msg__SDControl__Sequence *)allocator.allocate(sizeof(sd_msgs__msg__SDControl__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = sd_msgs__msg__SDControl__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
sd_msgs__msg__SDControl__Sequence__destroy(sd_msgs__msg__SDControl__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    sd_msgs__msg__SDControl__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
sd_msgs__msg__SDControl__Sequence__are_equal(const sd_msgs__msg__SDControl__Sequence * lhs, const sd_msgs__msg__SDControl__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!sd_msgs__msg__SDControl__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
sd_msgs__msg__SDControl__Sequence__copy(
  const sd_msgs__msg__SDControl__Sequence * input,
  sd_msgs__msg__SDControl__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(sd_msgs__msg__SDControl);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    sd_msgs__msg__SDControl * data =
      (sd_msgs__msg__SDControl *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!sd_msgs__msg__SDControl__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          sd_msgs__msg__SDControl__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!sd_msgs__msg__SDControl__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
