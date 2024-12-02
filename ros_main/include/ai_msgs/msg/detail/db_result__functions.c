// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ai_msgs:msg/DBResult.idl
// generated code does not contain a copyright notice
#include "ai_msgs/msg/detail/db_result__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `db_type`
// Member `match_id`
#include "rosidl_runtime_c/string_functions.h"

bool
ai_msgs__msg__DBResult__init(ai_msgs__msg__DBResult * msg)
{
  if (!msg) {
    return false;
  }
  // db_type
  if (!rosidl_runtime_c__String__init(&msg->db_type)) {
    ai_msgs__msg__DBResult__fini(msg);
    return false;
  }
  // match_id
  if (!rosidl_runtime_c__String__init(&msg->match_id)) {
    ai_msgs__msg__DBResult__fini(msg);
    return false;
  }
  // distance
  // similarity
  return true;
}

void
ai_msgs__msg__DBResult__fini(ai_msgs__msg__DBResult * msg)
{
  if (!msg) {
    return;
  }
  // db_type
  rosidl_runtime_c__String__fini(&msg->db_type);
  // match_id
  rosidl_runtime_c__String__fini(&msg->match_id);
  // distance
  // similarity
}

bool
ai_msgs__msg__DBResult__are_equal(const ai_msgs__msg__DBResult * lhs, const ai_msgs__msg__DBResult * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // db_type
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->db_type), &(rhs->db_type)))
  {
    return false;
  }
  // match_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->match_id), &(rhs->match_id)))
  {
    return false;
  }
  // distance
  if (lhs->distance != rhs->distance) {
    return false;
  }
  // similarity
  if (lhs->similarity != rhs->similarity) {
    return false;
  }
  return true;
}

bool
ai_msgs__msg__DBResult__copy(
  const ai_msgs__msg__DBResult * input,
  ai_msgs__msg__DBResult * output)
{
  if (!input || !output) {
    return false;
  }
  // db_type
  if (!rosidl_runtime_c__String__copy(
      &(input->db_type), &(output->db_type)))
  {
    return false;
  }
  // match_id
  if (!rosidl_runtime_c__String__copy(
      &(input->match_id), &(output->match_id)))
  {
    return false;
  }
  // distance
  output->distance = input->distance;
  // similarity
  output->similarity = input->similarity;
  return true;
}

ai_msgs__msg__DBResult *
ai_msgs__msg__DBResult__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ai_msgs__msg__DBResult * msg = (ai_msgs__msg__DBResult *)allocator.allocate(sizeof(ai_msgs__msg__DBResult), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ai_msgs__msg__DBResult));
  bool success = ai_msgs__msg__DBResult__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ai_msgs__msg__DBResult__destroy(ai_msgs__msg__DBResult * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ai_msgs__msg__DBResult__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ai_msgs__msg__DBResult__Sequence__init(ai_msgs__msg__DBResult__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ai_msgs__msg__DBResult * data = NULL;

  if (size) {
    data = (ai_msgs__msg__DBResult *)allocator.zero_allocate(size, sizeof(ai_msgs__msg__DBResult), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ai_msgs__msg__DBResult__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ai_msgs__msg__DBResult__fini(&data[i - 1]);
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
ai_msgs__msg__DBResult__Sequence__fini(ai_msgs__msg__DBResult__Sequence * array)
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
      ai_msgs__msg__DBResult__fini(&array->data[i]);
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

ai_msgs__msg__DBResult__Sequence *
ai_msgs__msg__DBResult__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ai_msgs__msg__DBResult__Sequence * array = (ai_msgs__msg__DBResult__Sequence *)allocator.allocate(sizeof(ai_msgs__msg__DBResult__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ai_msgs__msg__DBResult__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ai_msgs__msg__DBResult__Sequence__destroy(ai_msgs__msg__DBResult__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ai_msgs__msg__DBResult__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ai_msgs__msg__DBResult__Sequence__are_equal(const ai_msgs__msg__DBResult__Sequence * lhs, const ai_msgs__msg__DBResult__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ai_msgs__msg__DBResult__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ai_msgs__msg__DBResult__Sequence__copy(
  const ai_msgs__msg__DBResult__Sequence * input,
  ai_msgs__msg__DBResult__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ai_msgs__msg__DBResult);
    ai_msgs__msg__DBResult * data =
      (ai_msgs__msg__DBResult *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ai_msgs__msg__DBResult__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          ai_msgs__msg__DBResult__fini(&data[i]);
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
    if (!ai_msgs__msg__DBResult__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
