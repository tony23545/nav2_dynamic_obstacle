// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from nav2_dynamic_msgs:msg/Obstacle.idl
// generated code does not contain a copyright notice
#include "nav2_dynamic_msgs/msg/detail/obstacle__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `position`
#include "geometry_msgs/msg/detail/point__functions.h"
// Member `velocity`
#include "geometry_msgs/msg/detail/vector3__functions.h"

bool
nav2_dynamic_msgs__msg__Obstacle__init(nav2_dynamic_msgs__msg__Obstacle * msg)
{
  if (!msg) {
    return false;
  }
  // id
  // position
  if (!geometry_msgs__msg__Point__init(&msg->position)) {
    nav2_dynamic_msgs__msg__Obstacle__fini(msg);
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Vector3__init(&msg->velocity)) {
    nav2_dynamic_msgs__msg__Obstacle__fini(msg);
    return false;
  }
  return true;
}

void
nav2_dynamic_msgs__msg__Obstacle__fini(nav2_dynamic_msgs__msg__Obstacle * msg)
{
  if (!msg) {
    return;
  }
  // id
  // position
  geometry_msgs__msg__Point__fini(&msg->position);
  // velocity
  geometry_msgs__msg__Vector3__fini(&msg->velocity);
}

nav2_dynamic_msgs__msg__Obstacle *
nav2_dynamic_msgs__msg__Obstacle__create()
{
  nav2_dynamic_msgs__msg__Obstacle * msg = (nav2_dynamic_msgs__msg__Obstacle *)malloc(sizeof(nav2_dynamic_msgs__msg__Obstacle));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(nav2_dynamic_msgs__msg__Obstacle));
  bool success = nav2_dynamic_msgs__msg__Obstacle__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
nav2_dynamic_msgs__msg__Obstacle__destroy(nav2_dynamic_msgs__msg__Obstacle * msg)
{
  if (msg) {
    nav2_dynamic_msgs__msg__Obstacle__fini(msg);
  }
  free(msg);
}


bool
nav2_dynamic_msgs__msg__Obstacle__Sequence__init(nav2_dynamic_msgs__msg__Obstacle__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  nav2_dynamic_msgs__msg__Obstacle * data = NULL;
  if (size) {
    data = (nav2_dynamic_msgs__msg__Obstacle *)calloc(size, sizeof(nav2_dynamic_msgs__msg__Obstacle));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = nav2_dynamic_msgs__msg__Obstacle__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        nav2_dynamic_msgs__msg__Obstacle__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
nav2_dynamic_msgs__msg__Obstacle__Sequence__fini(nav2_dynamic_msgs__msg__Obstacle__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      nav2_dynamic_msgs__msg__Obstacle__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

nav2_dynamic_msgs__msg__Obstacle__Sequence *
nav2_dynamic_msgs__msg__Obstacle__Sequence__create(size_t size)
{
  nav2_dynamic_msgs__msg__Obstacle__Sequence * array = (nav2_dynamic_msgs__msg__Obstacle__Sequence *)malloc(sizeof(nav2_dynamic_msgs__msg__Obstacle__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = nav2_dynamic_msgs__msg__Obstacle__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
nav2_dynamic_msgs__msg__Obstacle__Sequence__destroy(nav2_dynamic_msgs__msg__Obstacle__Sequence * array)
{
  if (array) {
    nav2_dynamic_msgs__msg__Obstacle__Sequence__fini(array);
  }
  free(array);
}
