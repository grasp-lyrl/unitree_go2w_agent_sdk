// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from unitree_hg:msg/PressSensorState.idl
// generated code does not contain a copyright notice
#include "unitree_hg/msg/detail/press_sensor_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
unitree_hg__msg__PressSensorState__init(unitree_hg__msg__PressSensorState * msg)
{
  if (!msg) {
    return false;
  }
  // pressure
  // temperature
  return true;
}

void
unitree_hg__msg__PressSensorState__fini(unitree_hg__msg__PressSensorState * msg)
{
  if (!msg) {
    return;
  }
  // pressure
  // temperature
}

bool
unitree_hg__msg__PressSensorState__are_equal(const unitree_hg__msg__PressSensorState * lhs, const unitree_hg__msg__PressSensorState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // pressure
  for (size_t i = 0; i < 12; ++i) {
    if (lhs->pressure[i] != rhs->pressure[i]) {
      return false;
    }
  }
  // temperature
  for (size_t i = 0; i < 12; ++i) {
    if (lhs->temperature[i] != rhs->temperature[i]) {
      return false;
    }
  }
  return true;
}

bool
unitree_hg__msg__PressSensorState__copy(
  const unitree_hg__msg__PressSensorState * input,
  unitree_hg__msg__PressSensorState * output)
{
  if (!input || !output) {
    return false;
  }
  // pressure
  for (size_t i = 0; i < 12; ++i) {
    output->pressure[i] = input->pressure[i];
  }
  // temperature
  for (size_t i = 0; i < 12; ++i) {
    output->temperature[i] = input->temperature[i];
  }
  return true;
}

unitree_hg__msg__PressSensorState *
unitree_hg__msg__PressSensorState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_hg__msg__PressSensorState * msg = (unitree_hg__msg__PressSensorState *)allocator.allocate(sizeof(unitree_hg__msg__PressSensorState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(unitree_hg__msg__PressSensorState));
  bool success = unitree_hg__msg__PressSensorState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
unitree_hg__msg__PressSensorState__destroy(unitree_hg__msg__PressSensorState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    unitree_hg__msg__PressSensorState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
unitree_hg__msg__PressSensorState__Sequence__init(unitree_hg__msg__PressSensorState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_hg__msg__PressSensorState * data = NULL;

  if (size) {
    data = (unitree_hg__msg__PressSensorState *)allocator.zero_allocate(size, sizeof(unitree_hg__msg__PressSensorState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = unitree_hg__msg__PressSensorState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        unitree_hg__msg__PressSensorState__fini(&data[i - 1]);
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
unitree_hg__msg__PressSensorState__Sequence__fini(unitree_hg__msg__PressSensorState__Sequence * array)
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
      unitree_hg__msg__PressSensorState__fini(&array->data[i]);
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

unitree_hg__msg__PressSensorState__Sequence *
unitree_hg__msg__PressSensorState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_hg__msg__PressSensorState__Sequence * array = (unitree_hg__msg__PressSensorState__Sequence *)allocator.allocate(sizeof(unitree_hg__msg__PressSensorState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = unitree_hg__msg__PressSensorState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
unitree_hg__msg__PressSensorState__Sequence__destroy(unitree_hg__msg__PressSensorState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    unitree_hg__msg__PressSensorState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
unitree_hg__msg__PressSensorState__Sequence__are_equal(const unitree_hg__msg__PressSensorState__Sequence * lhs, const unitree_hg__msg__PressSensorState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!unitree_hg__msg__PressSensorState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
unitree_hg__msg__PressSensorState__Sequence__copy(
  const unitree_hg__msg__PressSensorState__Sequence * input,
  unitree_hg__msg__PressSensorState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(unitree_hg__msg__PressSensorState);
    unitree_hg__msg__PressSensorState * data =
      (unitree_hg__msg__PressSensorState *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!unitree_hg__msg__PressSensorState__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          unitree_hg__msg__PressSensorState__fini(&data[i]);
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
    if (!unitree_hg__msg__PressSensorState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
