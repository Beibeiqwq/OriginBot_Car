// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ai_msgs:msg/Point.idl
// generated code does not contain a copyright notice
#include "ai_msgs/msg/detail/point__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `type`
#include "rosidl_runtime_c/string_functions.h"
// Member `point`
#include "geometry_msgs/msg/detail/point32__functions.h"
// Member `confidence`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
ai_msgs__msg__Point__init(ai_msgs__msg__Point * msg)
{
  if (!msg) {
    return false;
  }
  // type
  if (!rosidl_runtime_c__String__init(&msg->type)) {
    ai_msgs__msg__Point__fini(msg);
    return false;
  }
  // point
  if (!geometry_msgs__msg__Point32__Sequence__init(&msg->point, 0)) {
    ai_msgs__msg__Point__fini(msg);
    return false;
  }
  // confidence
  if (!rosidl_runtime_c__float__Sequence__init(&msg->confidence, 0)) {
    ai_msgs__msg__Point__fini(msg);
    return false;
  }
  return true;
}

void
ai_msgs__msg__Point__fini(ai_msgs__msg__Point * msg)
{
  if (!msg) {
    return;
  }
  // type
  rosidl_runtime_c__String__fini(&msg->type);
  // point
  geometry_msgs__msg__Point32__Sequence__fini(&msg->point);
  // confidence
  rosidl_runtime_c__float__Sequence__fini(&msg->confidence);
}

bool
ai_msgs__msg__Point__are_equal(const ai_msgs__msg__Point * lhs, const ai_msgs__msg__Point * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // type
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->type), &(rhs->type)))
  {
    return false;
  }
  // point
  if (!geometry_msgs__msg__Point32__Sequence__are_equal(
      &(lhs->point), &(rhs->point)))
  {
    return false;
  }
  // confidence
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->confidence), &(rhs->confidence)))
  {
    return false;
  }
  return true;
}

bool
ai_msgs__msg__Point__copy(
  const ai_msgs__msg__Point * input,
  ai_msgs__msg__Point * output)
{
  if (!input || !output) {
    return false;
  }
  // type
  if (!rosidl_runtime_c__String__copy(
      &(input->type), &(output->type)))
  {
    return false;
  }
  // point
  if (!geometry_msgs__msg__Point32__Sequence__copy(
      &(input->point), &(output->point)))
  {
    return false;
  }
  // confidence
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->confidence), &(output->confidence)))
  {
    return false;
  }
  return true;
}

ai_msgs__msg__Point *
ai_msgs__msg__Point__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ai_msgs__msg__Point * msg = (ai_msgs__msg__Point *)allocator.allocate(sizeof(ai_msgs__msg__Point), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ai_msgs__msg__Point));
  bool success = ai_msgs__msg__Point__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ai_msgs__msg__Point__destroy(ai_msgs__msg__Point * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ai_msgs__msg__Point__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ai_msgs__msg__Point__Sequence__init(ai_msgs__msg__Point__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ai_msgs__msg__Point * data = NULL;

  if (size) {
    data = (ai_msgs__msg__Point *)allocator.zero_allocate(size, sizeof(ai_msgs__msg__Point), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ai_msgs__msg__Point__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ai_msgs__msg__Point__fini(&data[i - 1]);
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
ai_msgs__msg__Point__Sequence__fini(ai_msgs__msg__Point__Sequence * array)
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
      ai_msgs__msg__Point__fini(&array->data[i]);
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

ai_msgs__msg__Point__Sequence *
ai_msgs__msg__Point__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ai_msgs__msg__Point__Sequence * array = (ai_msgs__msg__Point__Sequence *)allocator.allocate(sizeof(ai_msgs__msg__Point__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ai_msgs__msg__Point__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ai_msgs__msg__Point__Sequence__destroy(ai_msgs__msg__Point__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ai_msgs__msg__Point__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ai_msgs__msg__Point__Sequence__are_equal(const ai_msgs__msg__Point__Sequence * lhs, const ai_msgs__msg__Point__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ai_msgs__msg__Point__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ai_msgs__msg__Point__Sequence__copy(
  const ai_msgs__msg__Point__Sequence * input,
  ai_msgs__msg__Point__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ai_msgs__msg__Point);
    ai_msgs__msg__Point * data =
      (ai_msgs__msg__Point *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ai_msgs__msg__Point__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          ai_msgs__msg__Point__fini(&data[i]);
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
    if (!ai_msgs__msg__Point__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
