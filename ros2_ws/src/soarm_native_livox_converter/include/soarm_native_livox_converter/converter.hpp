#pragma once

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace soarm_native_livox_converter
{

bool convert(
  const sensor_msgs::msg::PointCloud2 & source,
  sensor_msgs::msg::PointCloud2 & adapted,
  livox_ros_driver2::msg::CustomMsg & livox);

}  // namespace soarm_native_livox_converter
