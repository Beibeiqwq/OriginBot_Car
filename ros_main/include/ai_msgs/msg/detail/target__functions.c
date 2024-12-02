// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ai_msgs:msg/Target.idl
// generated code does not contain a copyright notice
#include "ai_msgs/msg/detail/target__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `type`
#include "rosidl_runtime_c/string_functions.h"
// Member `rois`
#include "ai_msgs/msg/detail/roi__functions.h"
// Member `attributes`
#include "ai_msgs/msg/detail/attribute__functions.h"
// Member `points`
#include "ai_msgs/msg/detail/point__functions.h"
// Member `captures`
#include "ai_msgs/msg/detail/capture__functions.h"

bool
ai_msgs__msg__Target__init(ai_msgs__msg__Target * msg)
{
  if (!msg) {
    return false;
  }
  // type
  if (!rosidl_runtime_c__String__init(&msg->type)) {
    ai_msgs__msg__Target__fini(msg);
    return false;
  }
  // track_id
  // rois
  if (!ai_msgs__msg__Roi__Sequence__init(&msg->rois, 0)) {
    ai_msgs__msg__Target__fini(msg);
    return false;
  }
  // attributes
  if (!ai_msgs__msg__Attribute__Sequence__init(&msg->attributes, 0)) {
    ai_msgs__msg__Target__fini(msg);
    return false;
  }
  // points
  if (!ai_msgs__msg__Point__Sequence__init(&msg->points, 0)) {
    ai_msgs__msg__Target__fini(msg);
    return false;
  }
  // captures
  if (!ai_msgs__msg__Capture__Sequence__init(&msg->captures, 0)) {
    ai_msgs__msg__Target__fini(msg);
    return false;
  }
  return true;
}

void
ai_msgs__msg__Target__fini(ai_msgs__msg__Target * msg)
{
  if (!msg) {
    return;
  }
  // type
  rosidl_runtime_c__String__fini(&msg->type);
  // track_id
  // rois
  ai_msgs__msg__Roi__Sequence__fini(&msg->rois);
  // attributes
  ai_msgs__msg__Attribute__Sequence__fini(&msg->attributes);
  // points
  ai_msgs__msg__Point__Sequence__fini(&msg->points);
  // captures
  ai_msgs__msg__Capture__Sequence__fini(&msg->captures);
}

bool
ai_msgs__msg__Target__are_equal(const ai_msgs__msg__Target * lhs, const ai_msgs__msg__Target * rhs)
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
  // track_id
  if (lhs->track_id != rhs->track_id) {
    return false;
  }
  // rois
  if (!ai_msgs__msg__Roi__Sequence__are_equal(
      &(lhs->rois), &(rhs->rois)))
  {
    return false;
  }
  // attributes
  if (!ai_msgs__msg__Attribute__Sequence__are_equal(
      &(lhs->attributes), &(rhs->attributes)))
  {
    return false;
  }
  // points
  if (!ai_msgs__msg__Point__Sequence__are_equal(
      &(lhs->points), &(rhs->points)))
  {
    return false;
  }
  // captures
  if (!ai_msgs__msg__Capture__Sequence__are_equal(
      &(lhs->captures), &(rhs->captures)))
  {
    return false;
  }
  return true;
}

bool
ai_msgs__msg__Target__copy(
  const ai_msgs__msg__Target * input,
  ai_msgs__msg__Target * output)
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
  // track_id
  output->track_id = input->track_id;
  // rois
  if (!ai_msgs__msg__Roi__Sequence__copy(
      &(input->rois), &(output->rois)))
  {
    return false;
  }
  // attributes
  if (!ai_msgs__msg__Attribute__Sequence__copy(
      &(input->attributes), &(output->attributes)))
  {
    return false;
  }
  // points
  if (!ai_msgs__msg__Point__Sequence__copy(
      &(input->points), &(output->points)))
  {
    return false;
  }
  // captures
  if (!ai_msgs__msg__Capture__Sequence__copy(
      &(input->captures), &(output->captures)))
  {
    return false;
  }
  return true;
}

ai_msgs__msg__Target *
ai_msgs__msg__Target__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ai_msgs__msg__Target * msg = (ai_msgs__msg__Target *)allocator.allocate(sizeof(ai_msgs__msg__Target), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ai_msgs__msg__Target));
  bool success = ai_msgs__msg__Target__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ai_msgs__msg__Target__destroy(ai_msgs__msg__Target * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ai_msgs__msg__Target__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ai_msgs__msg__Target__Sequence__init(ai_msgs__msg__Target__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ai_msgs__msg__Target * data = NULL;

  if (size) {
    data = (ai_msgs__msg__Target *)allocator.zero_allocate(size, sizeof(ai_msgs__msg__Target), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ai_msgs__msg__Target__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ai_msgs__msg__Target__fini(&data[i - 1]);
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
ai_msgs__msg__Target__Sequence__fini(ai_msgs__msg__Target__Sequence * array)
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
      ai_msgs__msg__Target__fini(&array->data[i]);
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

ai_msgs__msg__Target__Sequence *
ai_msgs__msg__Target__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ai_msgs__msg__Target__Sequence * array = (ai_msgs__msg__Target__Sequence *)allocator.allocate(sizeof(ai_msgs__msg__Target__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ai_msgs__msg__Target__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ai_msgs__msg__Target__Sequence__destroy(ai_msgs__msg__Target__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ai_msgs__msg__Target__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ai_msgs__msg__Target__Sequence__are_equal(const ai_msgs__msg__Target__Sequence * lhs, const ai_msgs__msg__Target__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ai_msgs__msg__Target__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ai_msgs__msg__Target__Sequence__copy(
  const ai_msgs__msg__Target__Sequence * input,
  ai_msgs__msg__Target__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ai_msgs__msg__Target);
    ai_msgs__msg__Target * data =
      (ai_msgs__msg__Target *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ai_msgs__msg__Target__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          ai_msgs__msg__Target__fini(&data[i]);
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
    if (!ai_msgs__msg__Target__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
