// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from nav2_dynamic_msgs:msg/ObstacleArray.idl
// generated code does not contain a copyright notice

#ifndef NAV2_DYNAMIC_MSGS__MSG__DETAIL__OBSTACLE_ARRAY__BUILDER_HPP_
#define NAV2_DYNAMIC_MSGS__MSG__DETAIL__OBSTACLE_ARRAY__BUILDER_HPP_

#include "nav2_dynamic_msgs/msg/detail/obstacle_array__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace nav2_dynamic_msgs
{

namespace msg
{

namespace builder
{

class Init_ObstacleArray_obstacles
{
public:
  explicit Init_ObstacleArray_obstacles(::nav2_dynamic_msgs::msg::ObstacleArray & msg)
  : msg_(msg)
  {}
  ::nav2_dynamic_msgs::msg::ObstacleArray obstacles(::nav2_dynamic_msgs::msg::ObstacleArray::_obstacles_type arg)
  {
    msg_.obstacles = std::move(arg);
    return std::move(msg_);
  }

private:
  ::nav2_dynamic_msgs::msg::ObstacleArray msg_;
};

class Init_ObstacleArray_header
{
public:
  Init_ObstacleArray_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ObstacleArray_obstacles header(::nav2_dynamic_msgs::msg::ObstacleArray::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ObstacleArray_obstacles(msg_);
  }

private:
  ::nav2_dynamic_msgs::msg::ObstacleArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::nav2_dynamic_msgs::msg::ObstacleArray>()
{
  return nav2_dynamic_msgs::msg::builder::Init_ObstacleArray_header();
}

}  // namespace nav2_dynamic_msgs

#endif  // NAV2_DYNAMIC_MSGS__MSG__DETAIL__OBSTACLE_ARRAY__BUILDER_HPP_
