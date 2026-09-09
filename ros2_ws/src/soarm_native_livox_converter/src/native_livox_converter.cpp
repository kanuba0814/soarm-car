#include "soarm_native_livox_converter/converter.hpp"

#include <rclcpp/rclcpp.hpp>

class NativeLivoxConverter : public rclcpp::Node
{
public:
  NativeLivoxConverter()
  : Node("native_livox_converter")
  {
    points_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/mid360/points", 10);
    livox_publisher_ = create_publisher<livox_ros_driver2::msg::CustomMsg>("/livox/lidar", 10);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/mid360/scan/points/points", 10,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr source) {
        sensor_msgs::msg::PointCloud2 adapted;
        livox_ros_driver2::msg::CustomMsg livox;
        if (!soarm_native_livox_converter::convert(*source, adapted, livox)) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "dropping PointCloud2 with unsupported layout");
          return;
        }
        points_publisher_->publish(adapted);
        livox_publisher_->publish(livox);
      });
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_publisher_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NativeLivoxConverter>());
  rclcpp::shutdown();
  return 0;
}
