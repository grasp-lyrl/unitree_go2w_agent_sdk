// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from unitree_hg:msg/LowCmd.idl
// generated code does not contain a copyright notice
#include "unitree_hg/msg/detail/low_cmd__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `motor_cmd`
#include "unitree_hg/msg/detail/motor_cmd__functions.h"

bool
unitree_hg__msg__LowCmd__init(unitree_hg__msg__LowCmd * msg)
{
  if (!msg) {
    return false;
  }
  // mode_pr
  // mode_machine
  // motor_cmd
  for (size_t i = 0; i < 35; ++i) {
    if (!unitree_hg__msg__MotorCmd__init(&msg->motor_cmd[i])) {
      unitree_hg__msg__LowCmd__fini(msg);
      return false;
    }
  }
  // reserve
  // crc
  return true;
}

void
unitree_hg__msg__LowCmd__fini(unitree_hg__msg__LowCmd * msg)
{
  if (!msg) {
    return;
  }
  // mode_pr
  // mode_machine
  // motor_cmd
  for (size_t i = 0; i < 35; ++i) {
    unitree_hg__msg__MotorCmd__fini(&msg->motor_cmd[i]);
  }
  // reserve
  // crc
}

bool
unitree_hg__msg__LowCmd__are_equal(const unitree_hg__msg__LowCmd * lhs, const unitree_hg__msg__LowCmd * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // mode_pr
  if (lhs->mode_pr != rhs->mode_pr) {
    return false;
  }
  // mode_machine
  if (lhs->mode_machine != rhs->mode_machine) {
    return false;
  }
  // motor_cmd
  for (size_t i = 0; i < 35; ++i) {
    if (!unitree_hg__msg__MotorCmd__are_equal(
        &(lhs->motor_cmd[i]), &(rhs->motor_cmd[i])))
    {
      return false;
    }
  }
  // reserve
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->reserve[i] != rhs->reserve[i]) {
      return false;
    }
  }
  // crc
  if (lhs->crc != rhs->crc) {
    return false;
  }
  return true;
}

bool
unitree_hg__msg__LowCmd__copy(
  const unitree_hg__msg__LowCmd * input,
  unitree_hg__msg__LowCmd * output)
{
  if (!input || !output) {
    return false;
  }
  // mode_pr
  output->mode_pr = input->mode_pr;
  // mode_machine
  output->mode_machine = input->mode_machine;
  // motor_cmd
  for (size_t i = 0; i < 35; ++i) {
    if (!unitree_hg__msg__MotorCmd__copy(
        &(input->motor_cmd[i]), &(output->motor_cmd[i])))
    {
      return false;
    }
  }
  // reserve
  for (size_t i = 0; i < 4; ++i) {
    output->reserve[i] = input->reserve[i];
  }
  // crc
  output->crc = input->crc;
  return true;
}

unitree_hg__msg__LowCmd *
unitree_hg__msg__LowCmd__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_hg__msg__LowCmd * msg = (unitree_hg__msg__LowCmd *)allocator.allocate(sizeof(unitree_hg__msg__LowCmd), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(unitree_hg__msg__LowCmd));
  bool success = unitree_hg__msg__LowCmd__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
unitree_hg__msg__LowCmd__destroy(unitree_hg__msg__LowCmd * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    unitree_hg__msg__LowCmd__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
unitree_hg__msg__LowCmd__Sequence__init(unitree_hg__msg__LowCmd__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_hg__msg__LowCmd * data = NULL;

  if (size) {
    data = (unitree_hg__msg__LowCmd *)allocator.zero_allocate(size, sizeof(unitree_hg__msg__LowCmd), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = unitree_hg__msg__LowCmd__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        unitree_hg__msg__LowCmd__fini(&data[i - 1]);
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
unitree_hg__msg__LowCmd__Sequence__fini(unitree_hg__msg__LowCmd__Sequence * array)
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
      unitree_hg__msg__LowCmd__fini(&array->data[i]);
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

unitree_hg__msg__LowCmd__Sequence *
unitree_hg__msg__LowCmd__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_hg__msg__LowCmd__Sequence * array = (unitree_hg__msg__LowCmd__Sequence *)allocator.allocate(sizeof(unitree_hg__msg__LowCmd__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = unitree_hg__msg__LowCmd__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
unitree_hg__msg__LowCmd__Sequence__destroy(unitree_hg__msg__LowCmd__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    unitree_hg__msg__LowCmd__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
unitree_hg__msg__LowCmd__Sequence__are_equal(const unitree_hg__msg__LowCmd__Sequence * lhs, const unitree_hg__msg__LowCmd__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!unitree_hg__msg__LowCmd__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
unitree_hg__msg__LowCmd__Sequence__copy(
  const unitree_hg__msg__LowCmd__Sequence * input,
  unitree_hg__msg__LowCmd__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(unitree_hg__msg__LowCmd);
    unitree_hg__msg__LowCmd * data =
      (unitree_hg__msg__LowCmd *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!unitree_hg__msg__LowCmd__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          unitree_hg__msg__LowCmd__fini(&data[i]);
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
    if (!unitree_hg__msg__LowCmd__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
