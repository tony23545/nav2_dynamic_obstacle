// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from nav2_dynamic_msgs:msg/ObstacleArray.idl
// generated code does not contain a copyright notice

#ifndef NAV2_DYNAMIC_MSGS__MSG__DETAIL__OBSTACLE_ARRAY__TRAITS_HPP_
#define NAV2_DYNAMIC_MSGS__MSG__DETAIL__OBSTACLE_ARRAY__TRAITS_HPP_

#include "nav2_dynamic_msgs/msg/detail/obstacle_array__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<nav2_dynamic_msgs::msg::ObstacleArray>()
{
  return "nav2_dynamic_msgs::msg::ObstacleArray";
}

template<>
inline const char * name<nav2_dynamic_msgs::msg::ObstacleArray>()
{
  return "nav2_dynamic_msgs/msg/ObstacleArray";
}

template<>
struct has_fixed_size<nav2_dynamic_msgs::msg::ObstacleArray>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<nav2_dynamic_msgs::msg::ObstacleArray>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<nav2_dynamic_msgs::msg::ObstacleArray>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // NAV2_DYNAMIC_MSGS__MSG__DETAIL__OBSTACLE_ARRAY__TRAITS_HPP_
