// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from unitree_go:msg/MotorStates.idl
// generated code does not contain a copyright notice
#include "unitree_go/msg/detail/motor_states__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `states`
#include "unitree_go/msg/detail/motor_state__functions.h"

bool
unitree_go__msg__MotorStates__init(unitree_go__msg__MotorStates * msg)
{
  if (!msg) {
    return false;
  }
  // states
  if (!unitree_go__msg__MotorState__Sequence__init(&msg->states, 0)) {
    unitree_go__msg__MotorStates__fini(msg);
    return false;
  }
  return true;
}

void
unitree_go__msg__MotorStates__fini(unitree_go__msg__MotorStates * msg)
{
  if (!msg) {
    return;
  }
  // states
  unitree_go__msg__MotorState__Sequence__fini(&msg->states);
}

bool
unitree_go__msg__MotorStates__are_equal(const unitree_go__msg__MotorStates * lhs, const unitree_go__msg__MotorStates * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // states
  if (!unitree_go__msg__MotorState__Sequence__are_equal(
      &(lhs->states), &(rhs->states)))
  {
    return false;
  }
  return true;
}

bool
unitree_go__msg__MotorStates__copy(
  const unitree_go__msg__MotorStates * input,
  unitree_go__msg__MotorStates * output)
{
  if (!input || !output) {
    return false;
  }
  // states
  if (!unitree_go__msg__MotorState__Sequence__copy(
      &(input->states), &(output->states)))
  {
    return false;
  }
  return true;
}

unitree_go__msg__MotorStates *
unitree_go__msg__MotorStates__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_go__msg__MotorStates * msg = (unitree_go__msg__MotorStates *)allocator.allocate(sizeof(unitree_go__msg__MotorStates), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(unitree_go__msg__MotorStates));
  bool success = unitree_go__msg__MotorStates__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
unitree_go__msg__MotorStates__destroy(unitree_go__msg__MotorStates * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    unitree_go__msg__MotorStates__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
unitree_go__msg__MotorStates__Sequence__init(unitree_go__msg__MotorStates__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_go__msg__MotorStates * data = NULL;

  if (size) {
    data = (unitree_go__msg__MotorStates *)allocator.zero_allocate(size, sizeof(unitree_go__msg__MotorStates), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = unitree_go__msg__MotorStates__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        unitree_go__msg__MotorStates__fini(&data[i - 1]);
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
unitree_go__msg__MotorStates__Sequence__fini(unitree_go__msg__MotorStates__Sequence * array)
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
      unitree_go__msg__MotorStates__fini(&array->data[i]);
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

unitree_go__msg__MotorStates__Sequence *
unitree_go__msg__MotorStates__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_go__msg__MotorStates__Sequence * array = (unitree_go__msg__MotorStates__Sequence *)allocator.allocate(sizeof(unitree_go__msg__MotorStates__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = unitree_go__msg__MotorStates__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
unitree_go__msg__MotorStates__Sequence__destroy(unitree_go__msg__MotorStates__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    unitree_go__msg__MotorStates__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
unitree_go__msg__MotorStates__Sequence__are_equal(const unitree_go__msg__MotorStates__Sequence * lhs, const unitree_go__msg__MotorStates__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!unitree_go__msg__MotorStates__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
unitree_go__msg__MotorStates__Sequence__copy(
  const unitree_go__msg__MotorStates__Sequence * input,
  unitree_go__msg__MotorStates__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(unitree_go__msg__MotorStates);
    unitree_go__msg__MotorStates * data =
      (unitree_go__msg__MotorStates *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!unitree_go__msg__MotorStates__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          unitree_go__msg__MotorStates__fini(&data[i]);
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
    if (!unitree_go__msg__MotorStates__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
