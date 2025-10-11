// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from unitree_hg:msg/BmsState.idl
// generated code does not contain a copyright notice
#include "unitree_hg/msg/detail/bms_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
unitree_hg__msg__BmsState__init(unitree_hg__msg__BmsState * msg)
{
  if (!msg) {
    return false;
  }
  // version_high
  // version_low
  // fn
  // cell_vol
  // bmsvoltage
  // current
  // soc
  // soh
  // temperature
  // cycle
  // manufacturer_date
  // bmsstate
  // reserve
  return true;
}

void
unitree_hg__msg__BmsState__fini(unitree_hg__msg__BmsState * msg)
{
  if (!msg) {
    return;
  }
  // version_high
  // version_low
  // fn
  // cell_vol
  // bmsvoltage
  // current
  // soc
  // soh
  // temperature
  // cycle
  // manufacturer_date
  // bmsstate
  // reserve
}

bool
unitree_hg__msg__BmsState__are_equal(const unitree_hg__msg__BmsState * lhs, const unitree_hg__msg__BmsState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // version_high
  if (lhs->version_high != rhs->version_high) {
    return false;
  }
  // version_low
  if (lhs->version_low != rhs->version_low) {
    return false;
  }
  // fn
  if (lhs->fn != rhs->fn) {
    return false;
  }
  // cell_vol
  for (size_t i = 0; i < 40; ++i) {
    if (lhs->cell_vol[i] != rhs->cell_vol[i]) {
      return false;
    }
  }
  // bmsvoltage
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->bmsvoltage[i] != rhs->bmsvoltage[i]) {
      return false;
    }
  }
  // current
  if (lhs->current != rhs->current) {
    return false;
  }
  // soc
  if (lhs->soc != rhs->soc) {
    return false;
  }
  // soh
  if (lhs->soh != rhs->soh) {
    return false;
  }
  // temperature
  for (size_t i = 0; i < 12; ++i) {
    if (lhs->temperature[i] != rhs->temperature[i]) {
      return false;
    }
  }
  // cycle
  if (lhs->cycle != rhs->cycle) {
    return false;
  }
  // manufacturer_date
  if (lhs->manufacturer_date != rhs->manufacturer_date) {
    return false;
  }
  // bmsstate
  for (size_t i = 0; i < 5; ++i) {
    if (lhs->bmsstate[i] != rhs->bmsstate[i]) {
      return false;
    }
  }
  // reserve
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->reserve[i] != rhs->reserve[i]) {
      return false;
    }
  }
  return true;
}

bool
unitree_hg__msg__BmsState__copy(
  const unitree_hg__msg__BmsState * input,
  unitree_hg__msg__BmsState * output)
{
  if (!input || !output) {
    return false;
  }
  // version_high
  output->version_high = input->version_high;
  // version_low
  output->version_low = input->version_low;
  // fn
  output->fn = input->fn;
  // cell_vol
  for (size_t i = 0; i < 40; ++i) {
    output->cell_vol[i] = input->cell_vol[i];
  }
  // bmsvoltage
  for (size_t i = 0; i < 3; ++i) {
    output->bmsvoltage[i] = input->bmsvoltage[i];
  }
  // current
  output->current = input->current;
  // soc
  output->soc = input->soc;
  // soh
  output->soh = input->soh;
  // temperature
  for (size_t i = 0; i < 12; ++i) {
    output->temperature[i] = input->temperature[i];
  }
  // cycle
  output->cycle = input->cycle;
  // manufacturer_date
  output->manufacturer_date = input->manufacturer_date;
  // bmsstate
  for (size_t i = 0; i < 5; ++i) {
    output->bmsstate[i] = input->bmsstate[i];
  }
  // reserve
  for (size_t i = 0; i < 3; ++i) {
    output->reserve[i] = input->reserve[i];
  }
  return true;
}

unitree_hg__msg__BmsState *
unitree_hg__msg__BmsState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_hg__msg__BmsState * msg = (unitree_hg__msg__BmsState *)allocator.allocate(sizeof(unitree_hg__msg__BmsState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(unitree_hg__msg__BmsState));
  bool success = unitree_hg__msg__BmsState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
unitree_hg__msg__BmsState__destroy(unitree_hg__msg__BmsState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    unitree_hg__msg__BmsState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
unitree_hg__msg__BmsState__Sequence__init(unitree_hg__msg__BmsState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_hg__msg__BmsState * data = NULL;

  if (size) {
    data = (unitree_hg__msg__BmsState *)allocator.zero_allocate(size, sizeof(unitree_hg__msg__BmsState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = unitree_hg__msg__BmsState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        unitree_hg__msg__BmsState__fini(&data[i - 1]);
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
unitree_hg__msg__BmsState__Sequence__fini(unitree_hg__msg__BmsState__Sequence * array)
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
      unitree_hg__msg__BmsState__fini(&array->data[i]);
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

unitree_hg__msg__BmsState__Sequence *
unitree_hg__msg__BmsState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_hg__msg__BmsState__Sequence * array = (unitree_hg__msg__BmsState__Sequence *)allocator.allocate(sizeof(unitree_hg__msg__BmsState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = unitree_hg__msg__BmsState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
unitree_hg__msg__BmsState__Sequence__destroy(unitree_hg__msg__BmsState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    unitree_hg__msg__BmsState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
unitree_hg__msg__BmsState__Sequence__are_equal(const unitree_hg__msg__BmsState__Sequence * lhs, const unitree_hg__msg__BmsState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!unitree_hg__msg__BmsState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
unitree_hg__msg__BmsState__Sequence__copy(
  const unitree_hg__msg__BmsState__Sequence * input,
  unitree_hg__msg__BmsState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(unitree_hg__msg__BmsState);
    unitree_hg__msg__BmsState * data =
      (unitree_hg__msg__BmsState *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!unitree_hg__msg__BmsState__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          unitree_hg__msg__BmsState__fini(&data[i]);
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
    if (!unitree_hg__msg__BmsState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
