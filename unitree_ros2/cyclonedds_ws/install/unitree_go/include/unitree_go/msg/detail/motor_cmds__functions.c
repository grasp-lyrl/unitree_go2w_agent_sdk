// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from unitree_go:msg/MotorCmds.idl
// generated code does not contain a copyright notice
#include "unitree_go/msg/detail/motor_cmds__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `cmds`
#include "unitree_go/msg/detail/motor_cmd__functions.h"

bool
unitree_go__msg__MotorCmds__init(unitree_go__msg__MotorCmds * msg)
{
  if (!msg) {
    return false;
  }
  // cmds
  if (!unitree_go__msg__MotorCmd__Sequence__init(&msg->cmds, 0)) {
    unitree_go__msg__MotorCmds__fini(msg);
    return false;
  }
  return true;
}

void
unitree_go__msg__MotorCmds__fini(unitree_go__msg__MotorCmds * msg)
{
  if (!msg) {
    return;
  }
  // cmds
  unitree_go__msg__MotorCmd__Sequence__fini(&msg->cmds);
}

bool
unitree_go__msg__MotorCmds__are_equal(const unitree_go__msg__MotorCmds * lhs, const unitree_go__msg__MotorCmds * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // cmds
  if (!unitree_go__msg__MotorCmd__Sequence__are_equal(
      &(lhs->cmds), &(rhs->cmds)))
  {
    return false;
  }
  return true;
}

bool
unitree_go__msg__MotorCmds__copy(
  const unitree_go__msg__MotorCmds * input,
  unitree_go__msg__MotorCmds * output)
{
  if (!input || !output) {
    return false;
  }
  // cmds
  if (!unitree_go__msg__MotorCmd__Sequence__copy(
      &(input->cmds), &(output->cmds)))
  {
    return false;
  }
  return true;
}

unitree_go__msg__MotorCmds *
unitree_go__msg__MotorCmds__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_go__msg__MotorCmds * msg = (unitree_go__msg__MotorCmds *)allocator.allocate(sizeof(unitree_go__msg__MotorCmds), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(unitree_go__msg__MotorCmds));
  bool success = unitree_go__msg__MotorCmds__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
unitree_go__msg__MotorCmds__destroy(unitree_go__msg__MotorCmds * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    unitree_go__msg__MotorCmds__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
unitree_go__msg__MotorCmds__Sequence__init(unitree_go__msg__MotorCmds__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_go__msg__MotorCmds * data = NULL;

  if (size) {
    data = (unitree_go__msg__MotorCmds *)allocator.zero_allocate(size, sizeof(unitree_go__msg__MotorCmds), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = unitree_go__msg__MotorCmds__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        unitree_go__msg__MotorCmds__fini(&data[i - 1]);
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
unitree_go__msg__MotorCmds__Sequence__fini(unitree_go__msg__MotorCmds__Sequence * array)
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
      unitree_go__msg__MotorCmds__fini(&array->data[i]);
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

unitree_go__msg__MotorCmds__Sequence *
unitree_go__msg__MotorCmds__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_go__msg__MotorCmds__Sequence * array = (unitree_go__msg__MotorCmds__Sequence *)allocator.allocate(sizeof(unitree_go__msg__MotorCmds__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = unitree_go__msg__MotorCmds__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
unitree_go__msg__MotorCmds__Sequence__destroy(unitree_go__msg__MotorCmds__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    unitree_go__msg__MotorCmds__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
unitree_go__msg__MotorCmds__Sequence__are_equal(const unitree_go__msg__MotorCmds__Sequence * lhs, const unitree_go__msg__MotorCmds__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!unitree_go__msg__MotorCmds__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
unitree_go__msg__MotorCmds__Sequence__copy(
  const unitree_go__msg__MotorCmds__Sequence * input,
  unitree_go__msg__MotorCmds__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(unitree_go__msg__MotorCmds);
    unitree_go__msg__MotorCmds * data =
      (unitree_go__msg__MotorCmds *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!unitree_go__msg__MotorCmds__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          unitree_go__msg__MotorCmds__fini(&data[i]);
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
    if (!unitree_go__msg__MotorCmds__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
