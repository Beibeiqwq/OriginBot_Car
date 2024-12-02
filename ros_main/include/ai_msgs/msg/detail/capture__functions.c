// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ai_msgs:msg/Capture.idl
// generated code does not contain a copyright notice
#include "ai_msgs/msg/detail/capture__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `img`
#include "sensor_msgs/msg/detail/image__functions.h"
// Member `features`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `db_result`
#include "ai_msgs/msg/detail/db_result__functions.h"

bool
ai_msgs__msg__Capture__init(ai_msgs__msg__Capture * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    ai_msgs__msg__Capture__fini(msg);
    return false;
  }
  // img
  if (!sensor_msgs__msg__Image__init(&msg->img)) {
    ai_msgs__msg__Capture__fini(msg);
    return false;
  }
  // features
  if (!rosidl_runtime_c__float__Sequence__init(&msg->features, 0)) {
    ai_msgs__msg__Capture__fini(msg);
    return false;
  }
  // db_result
  if (!ai_msgs__msg__DBResult__init(&msg->db_result)) {
    ai_msgs__msg__Capture__fini(msg);
    return false;
  }
  return true;
}

void
ai_msgs__msg__Capture__fini(ai_msgs__msg__Capture * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // img
  sensor_msgs__msg__Image__fini(&msg->img);
  // features
  rosidl_runtime_c__float__Sequence__fini(&msg->features);
  // db_result
  ai_msgs__msg__DBResult__fini(&msg->db_result);
}

bool
ai_msgs__msg__Capture__are_equal(const ai_msgs__msg__Capture * lhs, const ai_msgs__msg__Capture * rhs)
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
  // img
  if (!sensor_msgs__msg__Image__are_equal(
      &(lhs->img), &(rhs->img)))
  {
    return false;
  }
  // features
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->features), &(rhs->features)))
  {
    return false;
  }
  // db_result
  if (!ai_msgs__msg__DBResult__are_equal(
      &(lhs->db_result), &(rhs->db_result)))
  {
    return false;
  }
  return true;
}

bool
ai_msgs__msg__Capture__copy(
  const ai_msgs__msg__Capture * input,
  ai_msgs__msg__Capture * output)
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
  // img
  if (!sensor_msgs__msg__Image__copy(
      &(input->img), &(output->img)))
  {
    return false;
  }
  // features
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->features), &(output->features)))
  {
    return false;
  }
  // db_result
  if (!ai_msgs__msg__DBResult__copy(
      &(input->db_result), &(output->db_result)))
  {
    return false;
  }
  return true;
}

ai_msgs__msg__Capture *
ai_msgs__msg__Capture__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ai_msgs__msg__Capture * msg = (ai_msgs__msg__Capture *)allocator.allocate(sizeof(ai_msgs__msg__Capture), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ai_msgs__msg__Capture));
  bool success = ai_msgs__msg__Capture__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ai_msgs__msg__Capture__destroy(ai_msgs__msg__Capture * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ai_msgs__msg__Capture__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ai_msgs__msg__Capture__Sequence__init(ai_msgs__msg__Capture__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ai_msgs__msg__Capture * data = NULL;

  if (size) {
    data = (ai_msgs__msg__Capture *)allocator.zero_allocate(size, sizeof(ai_msgs__msg__Capture), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ai_msgs__msg__Capture__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ai_msgs__msg__Capture__fini(&data[i - 1]);
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
ai_msgs__msg__Capture__Sequence__fini(ai_msgs__msg__Capture__Sequence * array)
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
      ai_msgs__msg__Capture__fini(&array->data[i]);
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

ai_msgs__msg__Capture__Sequence *
ai_msgs__msg__Capture__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ai_msgs__msg__Capture__Sequence * array = (ai_msgs__msg__Capture__Sequence *)allocator.allocate(sizeof(ai_msgs__msg__Capture__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ai_msgs__msg__Capture__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ai_msgs__msg__Capture__Sequence__destroy(ai_msgs__msg__Capture__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ai_msgs__msg__Capture__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ai_msgs__msg__Capture__Sequence__are_equal(const ai_msgs__msg__Capture__Sequence * lhs, const ai_msgs__msg__Capture__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ai_msgs__msg__Capture__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ai_msgs__msg__Capture__Sequence__copy(
  const ai_msgs__msg__Capture__Sequence * input,
  ai_msgs__msg__Capture__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ai_msgs__msg__Capture);
    ai_msgs__msg__Capture * data =
      (ai_msgs__msg__Capture *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ai_msgs__msg__Capture__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          ai_msgs__msg__Capture__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ai_msgs__msg__Capture__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
