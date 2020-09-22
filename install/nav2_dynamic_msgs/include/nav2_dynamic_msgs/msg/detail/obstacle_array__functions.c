// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from nav2_dynamic_msgs:msg/ObstacleArray.idl
// generated code does not contain a copyright notice
#include "nav2_dynamic_msgs/msg/detail/obstacle_array__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `obstacles`
#include "nav2_dynamic_msgs/msg/detail/obstacle__functions.h"

bool
nav2_dynamic_msgs__msg__ObstacleArray__init(nav2_dynamic_msgs__msg__ObstacleArray * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    nav2_dynamic_msgs__msg__ObstacleArray__fini(msg);
    return false;
  }
  // obstacles
  if (!nav2_dynamic_msgs__msg__Obstacle__Sequence__init(&msg->obstacles, 0)) {
    nav2_dynamic_msgs__msg__ObstacleArray__fini(msg);
    return false;
  }
  return true;
}

void
nav2_dynamic_msgs__msg__ObstacleArray__fini(nav2_dynamic_msgs__msg__ObstacleArray * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // obstacles
  nav2_dynamic_msgs__msg__Obstacle__Sequence__fini(&msg->obstacles);
}

nav2_dynamic_msgs__msg__ObstacleArray *
nav2_dynamic_msgs__msg__ObstacleArray__create()
{
  nav2_dynamic_msgs__msg__ObstacleArray * msg = (nav2_dynamic_msgs__msg__ObstacleArray *)malloc(sizeof(nav2_dynamic_msgs__msg__ObstacleArray));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(nav2_dynamic_msgs__msg__ObstacleArray));
  bool success = nav2_dynamic_msgs__msg__ObstacleArray__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
nav2_dynamic_msgs__msg__ObstacleArray__destroy(nav2_dynamic_msgs__msg__ObstacleArray * msg)
{
  if (msg) {
    nav2_dynamic_msgs__msg__ObstacleArray__fini(msg);
  }
  free(msg);
}


bool
nav2_dynamic_msgs__msg__ObstacleArray__Sequence__init(nav2_dynamic_msgs__msg__ObstacleArray__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  nav2_dynamic_msgs__msg__ObstacleArray * data = NULL;
  if (size) {
    data = (nav2_dynamic_msgs__msg__ObstacleArray *)calloc(size, sizeof(nav2_dynamic_msgs__msg__ObstacleArray));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = nav2_dynamic_msgs__msg__ObstacleArray__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        nav2_dynamic_msgs__msg__ObstacleArray__fini(&data[i - 1]);
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
nav2_dynamic_msgs__msg__ObstacleArray__Sequence__fini(nav2_dynamic_msgs__msg__ObstacleArray__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      nav2_dynamic_msgs__msg__ObstacleArray__fini(&array->data[i]);
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

nav2_dynamic_msgs__msg__ObstacleArray__Sequence *
nav2_dynamic_msgs__msg__ObstacleArray__Sequence__create(size_t size)
{
  nav2_dynamic_msgs__msg__ObstacleArray__Sequence * array = (nav2_dynamic_msgs__msg__ObstacleArray__Sequence *)malloc(sizeof(nav2_dynamic_msgs__msg__ObstacleArray__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = nav2_dynamic_msgs__msg__ObstacleArray__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
nav2_dynamic_msgs__msg__ObstacleArray__Sequence__destroy(nav2_dynamic_msgs__msg__ObstacleArray__Sequence * array)
{
  if (array) {
    nav2_dynamic_msgs__msg__ObstacleArray__Sequence__fini(array);
  }
  free(array);
}
