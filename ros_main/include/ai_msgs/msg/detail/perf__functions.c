// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ai_msgs:msg/Perf.idl
// generated code does not contain a copyright notice
#include "ai_msgs/msg/detail/perf__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `type`
#include "rosidl_runtime_c/string_functions.h"
// Member `stamp_start`
// Member `stamp_end`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
ai_msgs__msg__Perf__init(ai_msgs__msg__Perf * msg)
{
  if (!msg) {
    return false;
  }
  // type
  if (!rosidl_runtime_c__String__init(&msg->type)) {
    ai_msgs__msg__Perf__fini(msg);
    return false;
  }
  // stamp_start
  if (!builtin_interfaces__msg__Time__init(&msg->stamp_start)) {
    ai_msgs__msg__Perf__fini(msg);
    return false;
  }
  // stamp_end
  if (!builtin_interfaces__msg__Time__init(&msg->stamp_end)) {
    ai_msgs__msg__Perf__fini(msg);
    return false;
  }
  // time_ms_duration
  return true;
}

void
ai_msgs__msg__Perf__fini(ai_msgs__msg__Perf * msg)
{
  if (!msg) {
    return;
  }
  // type
  rosidl_runtime_c__String__fini(&msg->type);
  // stamp_start
  builtin_interfaces__msg__Time__fini(&msg->stamp_start);
  // stamp_end
  builtin_interfaces__msg__Time__fini(&msg->stamp_end);
  // time_ms_duration
}

bool
ai_msgs__msg__Perf__are_equal(const ai_msgs__msg__Perf * lhs, const ai_msgs__msg__Perf * rhs)
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
  // stamp_start
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp_start), &(rhs->stamp_start)))
  {
    return false;
  }
  // stamp_end
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp_end), &(rhs->stamp_end)))
  {
    return false;
  }
  // time_ms_duration
  if (lhs->time_ms_duration != rhs->time_ms_duration) {
    return false;
  }
  return true;
}

bool
ai_msgs__msg__Perf__copy(
  const ai_msgs__msg__Perf * input,
  ai_msgs__msg__Perf * output)
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
  // stamp_start
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp_start), &(output->stamp_start)))
  {
    return false;
  }
  // stamp_end
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp_end), &(output->stamp_end)))
  {
    return false;
  }
  // time_ms_duration
  output->time_ms_duration = input->time_ms_duration;
  return true;
}

ai_msgs__msg__Perf *
ai_msgs__msg__Perf__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ai_msgs__msg__Perf * msg = (ai_msgs__msg__Perf *)allocator.allocate(sizeof(ai_msgs__msg__Perf), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ai_msgs__msg__Perf));
  bool success = ai_msgs__msg__Perf__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ai_msgs__msg__Perf__destroy(ai_msgs__msg__Perf * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ai_msgs__msg__Perf__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ai_msgs__msg__Perf__Sequence__init(ai_msgs__msg__Perf__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ai_msgs__msg__Perf * data = NULL;

  if (size) {
    data = (ai_msgs__msg__Perf *)allocator.zero_allocate(size, sizeof(ai_msgs__msg__Perf), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ai_msgs__msg__Perf__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ai_msgs__msg__Perf__fini(&data[i - 1]);
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
ai_msgs__msg__Perf__Sequence__fini(ai_msgs__msg__Perf__Sequence * array)
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
      ai_msgs__msg__Perf__fini(&array->data[i]);
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

ai_msgs__msg__Perf__Sequence *
ai_msgs__msg__Perf__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ai_msgs__msg__Perf__Sequence * array = (ai_msgs__msg__Perf__Sequence *)allocator.allocate(sizeof(ai_msgs__msg__Perf__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ai_msgs__msg__Perf__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ai_msgs__msg__Perf__Sequence__destroy(ai_msgs__msg__Perf__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ai_msgs__msg__Perf__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ai_msgs__msg__Perf__Sequence__are_equal(const ai_msgs__msg__Perf__Sequence * lhs, const ai_msgs__msg__Perf__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ai_msgs__msg__Perf__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ai_msgs__msg__Perf__Sequence__copy(
  const ai_msgs__msg__Perf__Sequence * input,
  ai_msgs__msg__Perf__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ai_msgs__msg__Perf);
    ai_msgs__msg__Perf * data =
      (ai_msgs__msg__Perf *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ai_msgs__msg__Perf__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          ai_msgs__msg__Perf__fini(&data[i]);
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
    if (!ai_msgs__msg__Perf__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}