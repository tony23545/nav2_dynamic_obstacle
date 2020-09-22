// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from nav2_dynamic_msgs:msg/Obstacle.idl
// generated code does not contain a copyright notice

#ifndef NAV2_DYNAMIC_MSGS__MSG__DETAIL__OBSTACLE__TRAITS_HPP_
#define NAV2_DYNAMIC_MSGS__MSG__DETAIL__OBSTACLE__TRAITS_HPP_

#include "nav2_dynamic_msgs/msg/detail/obstacle__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'position'
#include "geometry_msgs/msg/detail/point__traits.hpp"
// Member 'velocity'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<nav2_dynamic_msgs::msg::Obstacle>()
{
  return "nav2_dynamic_msgs::msg::Obstacle";
}

template<>
inline const char * name<nav2_dynamic_msgs::msg::Obstacle>()
{
  return "nav2_dynamic_msgs/msg/Obstacle";
}

template<>
struct has_fixed_size<nav2_dynamic_msgs::msg::Obstacle>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Point>::value && has_fixed_size<geometry_msgs::msg::Vector3>::value> {};

template<>
struct has_bounded_size<nav2_dynamic_msgs::msg::Obstacle>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Point>::value && has_bounded_size<geometry_msgs::msg::Vector3>::value> {};

template<>
struct is_message<nav2_dynamic_msgs::msg::Obstacle>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // NAV2_DYNAMIC_MSGS__MSG__DETAIL__OBSTACLE__TRAITS_HPP_
