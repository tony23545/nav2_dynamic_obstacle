// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from nav2_dynamic_msgs:msg/Obstacle.idl
// generated code does not contain a copyright notice

#ifndef NAV2_DYNAMIC_MSGS__MSG__DETAIL__OBSTACLE__FUNCTIONS_H_
#define NAV2_DYNAMIC_MSGS__MSG__DETAIL__OBSTACLE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "nav2_dynamic_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "nav2_dynamic_msgs/msg/detail/obstacle__struct.h"

/// Initialize msg/Obstacle message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * nav2_dynamic_msgs__msg__Obstacle
 * )) before or use
 * nav2_dynamic_msgs__msg__Obstacle__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_nav2_dynamic_msgs
bool
nav2_dynamic_msgs__msg__Obstacle__init(nav2_dynamic_msgs__msg__Obstacle * msg);

/// Finalize msg/Obstacle message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_nav2_dynamic_msgs
void
nav2_dynamic_msgs__msg__Obstacle__fini(nav2_dynamic_msgs__msg__Obstacle * msg);

/// Create msg/Obstacle message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * nav2_dynamic_msgs__msg__Obstacle__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_nav2_dynamic_msgs
nav2_dynamic_msgs__msg__Obstacle *
nav2_dynamic_msgs__msg__Obstacle__create();

/// Destroy msg/Obstacle message.
/**
 * It calls
 * nav2_dynamic_msgs__msg__Obstacle__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_nav2_dynamic_msgs
void
nav2_dynamic_msgs__msg__Obstacle__destroy(nav2_dynamic_msgs__msg__Obstacle * msg);


/// Initialize array of msg/Obstacle messages.
/**
 * It allocates the memory for the number of elements and calls
 * nav2_dynamic_msgs__msg__Obstacle__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_nav2_dynamic_msgs
bool
nav2_dynamic_msgs__msg__Obstacle__Sequence__init(nav2_dynamic_msgs__msg__Obstacle__Sequence * array, size_t size);

/// Finalize array of msg/Obstacle messages.
/**
 * It calls
 * nav2_dynamic_msgs__msg__Obstacle__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_nav2_dynamic_msgs
void
nav2_dynamic_msgs__msg__Obstacle__Sequence__fini(nav2_dynamic_msgs__msg__Obstacle__Sequence * array);

/// Create array of msg/Obstacle messages.
/**
 * It allocates the memory for the array and calls
 * nav2_dynamic_msgs__msg__Obstacle__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_nav2_dynamic_msgs
nav2_dynamic_msgs__msg__Obstacle__Sequence *
nav2_dynamic_msgs__msg__Obstacle__Sequence__create(size_t size);

/// Destroy array of msg/Obstacle messages.
/**
 * It calls
 * nav2_dynamic_msgs__msg__Obstacle__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_nav2_dynamic_msgs
void
nav2_dynamic_msgs__msg__Obstacle__Sequence__destroy(nav2_dynamic_msgs__msg__Obstacle__Sequence * array);

#ifdef __cplusplus
}
#endif

#endif  // NAV2_DYNAMIC_MSGS__MSG__DETAIL__OBSTACLE__FUNCTIONS_H_
