// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from nav2_dynamic_msgs:msg/Obstacle.idl
// generated code does not contain a copyright notice

#ifndef NAV2_DYNAMIC_MSGS__MSG__DETAIL__OBSTACLE__STRUCT_H_
#define NAV2_DYNAMIC_MSGS__MSG__DETAIL__OBSTACLE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.h"
// Member 'velocity'
#include "geometry_msgs/msg/detail/vector3__struct.h"

// Struct defined in msg/Obstacle in the package nav2_dynamic_msgs.
typedef struct nav2_dynamic_msgs__msg__Obstacle
{
  int8_t id;
  geometry_msgs__msg__Point position;
  geometry_msgs__msg__Vector3 velocity;
} nav2_dynamic_msgs__msg__Obstacle;

// Struct for a sequence of nav2_dynamic_msgs__msg__Obstacle.
typedef struct nav2_dynamic_msgs__msg__Obstacle__Sequence
{
  nav2_dynamic_msgs__msg__Obstacle * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} nav2_dynamic_msgs__msg__Obstacle__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // NAV2_DYNAMIC_MSGS__MSG__DETAIL__OBSTACLE__STRUCT_H_
