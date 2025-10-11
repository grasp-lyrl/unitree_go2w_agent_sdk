// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from unitree_hg:msg/LowState.idl
// generated code does not contain a copyright notice
#include "unitree_hg/msg/detail/low_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `imu_state`
#include "unitree_hg/msg/detail/imu_state__functions.h"
// Member `motor_state`
#include "unitree_hg/msg/detail/motor_state__functions.h"

bool
unitree_hg__msg__LowState__init(unitree_hg__msg__LowState * msg)
{
  if (!msg) {
    return false;
  }
  // version
  // mode_pr
  // mode_machine
  // tick
  // imu_state
  if (!unitree_hg__msg__IMUState__init(&msg->imu_state)) {
    unitree_hg__msg__LowState__fini(msg);
    return false;
  }
  // motor_state
  for (size_t i = 0; i < 35; ++i) {
    if (!unitree_hg__msg__MotorState__init(&msg->motor_state[i])) {
      unitree_hg__msg__LowState__fini(msg);
      return false;
    }
  }
  // wireless_remote
  // reserve
  // crc
  return true;
}

void
unitree_hg__msg__LowState__fini(unitree_hg__msg__LowState * msg)
{
  if (!msg) {
    return;
  }
  // version
  // mode_pr
  // mode_machine
  // tick
  // imu_state
  unitree_hg__msg__IMUState__fini(&msg->imu_state);
  // motor_state
  for (size_t i = 0; i < 35; ++i) {
    unitree_hg__msg__MotorState__fini(&msg->motor_state[i]);
  }
  // wireless_remote
  // reserve
  // crc
}

bool
unitree_hg__msg__LowState__are_equal(const unitree_hg__msg__LowState * lhs, const unitree_hg__msg__LowState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // version
  for (size_t i = 0; i < 2; ++i) {
    if (lhs->version[i] != rhs->version[i]) {
      return false;
    }
  }
  // mode_pr
  if (lhs->mode_pr != rhs->mode_pr) {
    return false;
  }
  // mode_machine
  if (lhs->mode_machine != rhs->mode_machine) {
    return false;
  }
  // tick
  if (lhs->tick != rhs->tick) {
    return false;
  }
  // imu_state
  if (!unitree_hg__msg__IMUState__are_equal(
      &(lhs->imu_state), &(rhs->imu_state)))
  {
    return false;
  }
  // motor_state
  for (size_t i = 0; i < 35; ++i) {
    if (!unitree_hg__msg__MotorState__are_equal(
        &(lhs->motor_state[i]), &(rhs->motor_state[i])))
    {
      return false;
    }
  }
  // wireless_remote
  for (size_t i = 0; i < 40; ++i) {
    if (lhs->wireless_remote[i] != rhs->wireless_remote[i]) {
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
unitree_hg__msg__LowState__copy(
  const unitree_hg__msg__LowState * input,
  unitree_hg__msg__LowState * output)
{
  if (!input || !output) {
    return false;
  }
  // version
  for (size_t i = 0; i < 2; ++i) {
    output->version[i] = input->version[i];
  }
  // mode_pr
  output->mode_pr = input->mode_pr;
  // mode_machine
  output->mode_machine = input->mode_machine;
  // tick
  output->tick = input->tick;
  // imu_state
  if (!unitree_hg__msg__IMUState__copy(
      &(input->imu_state), &(output->imu_state)))
  {
    return false;
  }
  // motor_state
  for (size_t i = 0; i < 35; ++i) {
    if (!unitree_hg__msg__MotorState__copy(
        &(input->motor_state[i]), &(output->motor_state[i])))
    {
      return false;
    }
  }
  // wireless_remote
  for (size_t i = 0; i < 40; ++i) {
    output->wireless_remote[i] = input->wireless_remote[i];
  }
  // reserve
  for (size_t i = 0; i < 4; ++i) {
    output->reserve[i] = input->reserve[i];
  }
  // crc
  output->crc = input->crc;
  return true;
}

unitree_hg__msg__LowState *
unitree_hg__msg__LowState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_hg__msg__LowState * msg = (unitree_hg__msg__LowState *)allocator.allocate(sizeof(unitree_hg__msg__LowState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(unitree_hg__msg__LowState));
  bool success = unitree_hg__msg__LowState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
unitree_hg__msg__LowState__destroy(unitree_hg__msg__LowState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    unitree_hg__msg__LowState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
unitree_hg__msg__LowState__Sequence__init(unitree_hg__msg__LowState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_hg__msg__LowState * data = NULL;

  if (size) {
    data = (unitree_hg__msg__LowState *)allocator.zero_allocate(size, sizeof(unitree_hg__msg__LowState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = unitree_hg__msg__LowState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        unitree_hg__msg__LowState__fini(&data[i - 1]);
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
unitree_hg__msg__LowState__Sequence__fini(unitree_hg__msg__LowState__Sequence * array)
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
      unitree_hg__msg__LowState__fini(&array->data[i]);
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

unitree_hg__msg__LowState__Sequence *
unitree_hg__msg__LowState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_hg__msg__LowState__Sequence * array = (unitree_hg__msg__LowState__Sequence *)allocator.allocate(sizeof(unitree_hg__msg__LowState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = unitree_hg__msg__LowState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
unitree_hg__msg__LowState__Sequence__destroy(unitree_hg__msg__LowState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    unitree_hg__msg__LowState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
unitree_hg__msg__LowState__Sequence__are_equal(const unitree_hg__msg__LowState__Sequence * lhs, const unitree_hg__msg__LowState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!unitree_hg__msg__LowState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
unitree_hg__msg__LowState__Sequence__copy(
  const unitree_hg__msg__LowState__Sequence * input,
  unitree_hg__msg__LowState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(unitree_hg__msg__LowState);
    unitree_hg__msg__LowState * data =
      (unitree_hg__msg__LowState *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!unitree_hg__msg__LowState__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          unitree_hg__msg__LowState__fini(&data[i]);
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
    if (!unitree_hg__msg__LowState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
