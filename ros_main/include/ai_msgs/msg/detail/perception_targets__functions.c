// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ai_msgs:msg/PerceptionTargets.idl
// generated code does not contain a copyright notice
#include "ai_msgs/msg/detail/perception_targets__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `perfs`
#include "ai_msgs/msg/detail/perf__functions.h"
// Member `targets`
// Member `disappeared_targets`
#include "ai_msgs/msg/detail/target__functions.h"

bool
ai_msgs__msg__PerceptionTargets__init(ai_msgs__msg__PerceptionTargets * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    ai_msgs__msg__PerceptionTargets__fini(msg);
    return false;
  }
  // fps
  // perfs
  if (!ai_msgs__msg__Perf__Sequence__init(&msg->perfs, 0)) {
    ai_msgs__msg__PerceptionTargets__fini(msg);
    return false;
  }
  // targets
  if (!ai_msgs__msg__Target__Sequence__init(&msg->targets, 0)) {
    ai_msgs__msg__PerceptionTargets__fini(msg);
    return false;
  }
  // disappeared_targets
  if (!ai_msgs__msg__Target__Sequence__init(&msg->disappeared_targets, 0)) {
    ai_msgs__msg__PerceptionTargets__fini(msg);
    return false;
  }
  return true;
}

void
ai_msgs__msg__PerceptionTargets__fini(ai_msgs__msg__PerceptionTargets * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // fps
  // perfs
  ai_msgs__msg__Perf__Sequence__fini(&msg->perfs);
  // targets
  ai_msgs__msg__Target__Sequence__fini(&msg->targets);
  // disappeared_targets
  ai_msgs__msg__Target__Sequence__fini(&msg->disappeared_targets);
}

bool
ai_msgs__msg__PerceptionTargets__are_equal(const ai_msgs__msg__PerceptionTargets * lhs, const ai_msgs__msg__PerceptionTargets * rhs)
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
  // fps
  if (lhs->fps != rhs->fps) {
    return false;
  }
  // perfs
  if (!ai_msgs__msg__Perf__Sequence__are_equal(
      &(lhs->perfs), &(rhs->perfs)))
  {
    return false;
  }
  // targets
  if (!ai_msgs__msg__Target__Sequence__are_equal(
      &(lhs->targets), &(rhs->targets)))
  {
    return false;
  }
  // disappeared_targets
  if (!ai_msgs__msg__Target__Sequence__are_equal(
      &(lhs->disappeared_targets), &(rhs->disappeared_targets)))
  {
    return false;
  }
  return true;
}

bool
ai_msgs__msg__PerceptionTargets__copy(
  const ai_msgs__msg__PerceptionTargets * input,
  ai_msgs__msg__PerceptionTargets * output)
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
  // fps
  output->fps = input->fps;
  // perfs
  if (!ai_msgs__msg__Perf__Sequence__copy(
      &(input->perfs), &(output->perfs)))
  {
    return false;
  }
  // targets
  if (!ai_msgs__msg__Target__Sequence__copy(
      &(input->targets), &(output->targets)))
  {
    return false;
  }
  // disappeared_targets
  if (!ai_msgs__msg__Target__Sequence__copy(
      &(input->disappeared_targets), &(output->disappeared_targets)))
  {
    return false;
  }
  return true;
}

ai_msgs__msg__PerceptionTargets *
ai_msgs__msg__PerceptionTargets__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ai_msgs__msg__PerceptionTargets * msg = (ai_msgs__msg__PerceptionTargets *)allocator.allocate(sizeof(ai_msgs__msg__PerceptionTargets), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ai_msgs__msg__PerceptionTargets));
  bool success = ai_msgs__msg__PerceptionTargets__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ai_msgs__msg__PerceptionTargets__destroy(ai_msgs__msg__PerceptionTargets * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ai_msgs__msg__PerceptionTargets__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ai_msgs__msg__PerceptionTargets__Sequence__init(ai_msgs__msg__PerceptionTargets__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ai_msgs__msg__PerceptionTargets * data = NULL;

  if (size) {
    data = (ai_msgs__msg__PerceptionTargets *)allocator.zero_allocate(size, sizeof(ai_msgs__msg__PerceptionTargets), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ai_msgs__msg__PerceptionTargets__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ai_msgs__msg__PerceptionTargets__fini(&data[i - 1]);
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
ai_msgs__msg__PerceptionTargets__Sequence__fini(ai_msgs__msg__PerceptionTargets__Sequence * array)
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
      ai_msgs__msg__PerceptionTargets__fini(&array->data[i]);
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

ai_msgs__msg__PerceptionTargets__Sequence *
ai_msgs__msg__PerceptionTargets__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ai_msgs__msg__PerceptionTargets__Sequence * array = (ai_msgs__msg__PerceptionTargets__Sequence *)allocator.allocate(sizeof(ai_msgs__msg__PerceptionTargets__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ai_msgs__msg__PerceptionTargets__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ai_msgs__msg__PerceptionTargets__Sequence__destroy(ai_msgs__msg__PerceptionTargets__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ai_msgs__msg__PerceptionTargets__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ai_msgs__msg__PerceptionTargets__Sequence__are_equal(const ai_msgs__msg__PerceptionTargets__Sequence * lhs, const ai_msgs__msg__PerceptionTargets__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ai_msgs__msg__PerceptionTargets__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ai_msgs__msg__PerceptionTargets__Sequence__copy(
  const ai_msgs__msg__PerceptionTargets__Sequence * input,
  ai_msgs__msg__PerceptionTargets__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ai_msgs__msg__PerceptionTargets);
    ai_msgs__msg__PerceptionTargets * data =
      (ai_msgs__msg__PerceptionTargets *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ai_msgs__msg__PerceptionTargets__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          ai_msgs__msg__PerceptionTargets__fini(&data[i]);
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
    if (!ai_msgs__msg__PerceptionTargets__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}