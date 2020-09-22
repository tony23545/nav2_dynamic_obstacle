// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from nav2_dynamic_msgs:msg/Obstacle.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "nav2_dynamic_msgs/msg/detail/obstacle__rosidl_typesupport_introspection_c.h"
#include "nav2_dynamic_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "nav2_dynamic_msgs/msg/detail/obstacle__functions.h"
#include "nav2_dynamic_msgs/msg/detail/obstacle__struct.h"


// Include directives for member types
// Member `position`
#include "geometry_msgs/msg/point.h"
// Member `position`
#include "geometry_msgs/msg/detail/point__rosidl_typesupport_introspection_c.h"
// Member `velocity`
#include "geometry_msgs/msg/vector3.h"
// Member `velocity`
#include "geometry_msgs/msg/detail/vector3__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void Obstacle__rosidl_typesupport_introspection_c__Obstacle_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  nav2_dynamic_msgs__msg__Obstacle__init(message_memory);
}

void Obstacle__rosidl_typesupport_introspection_c__Obstacle_fini_function(void * message_memory)
{
  nav2_dynamic_msgs__msg__Obstacle__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember Obstacle__rosidl_typesupport_introspection_c__Obstacle_message_member_array[3] = {
  {
    "id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(nav2_dynamic_msgs__msg__Obstacle, id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(nav2_dynamic_msgs__msg__Obstacle, position),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(nav2_dynamic_msgs__msg__Obstacle, velocity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers Obstacle__rosidl_typesupport_introspection_c__Obstacle_message_members = {
  "nav2_dynamic_msgs__msg",  // message namespace
  "Obstacle",  // message name
  3,  // number of fields
  sizeof(nav2_dynamic_msgs__msg__Obstacle),
  Obstacle__rosidl_typesupport_introspection_c__Obstacle_message_member_array,  // message members
  Obstacle__rosidl_typesupport_introspection_c__Obstacle_init_function,  // function to initialize message memory (memory has to be allocated)
  Obstacle__rosidl_typesupport_introspection_c__Obstacle_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t Obstacle__rosidl_typesupport_introspection_c__Obstacle_message_type_support_handle = {
  0,
  &Obstacle__rosidl_typesupport_introspection_c__Obstacle_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_nav2_dynamic_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, nav2_dynamic_msgs, msg, Obstacle)() {
  Obstacle__rosidl_typesupport_introspection_c__Obstacle_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  Obstacle__rosidl_typesupport_introspection_c__Obstacle_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  if (!Obstacle__rosidl_typesupport_introspection_c__Obstacle_message_type_support_handle.typesupport_identifier) {
    Obstacle__rosidl_typesupport_introspection_c__Obstacle_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &Obstacle__rosidl_typesupport_introspection_c__Obstacle_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
