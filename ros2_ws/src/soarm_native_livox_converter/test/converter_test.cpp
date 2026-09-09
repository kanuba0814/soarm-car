#include "soarm_native_livox_converter/converter.hpp"

#include <gtest/gtest.h>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include <cmath>
#include <cstring>
#include <fstream>
#include <iterator>
#include <limits>
#include <vector>

namespace
{
sensor_msgs::msg::PointCloud2 read_captured_cloud()
{
  std::ifstream input(RAW_CLOUD_CDR_PATH, std::ios::binary);
  std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
  EXPECT_FALSE(bytes.empty());
  rclcpp::SerializedMessage serialized(bytes.size());
  auto & message = serialized.get_rcl_serialized_message();
  std::memcpy(message.buffer, bytes.data(), bytes.size());
  message.buffer_length = bytes.size();
  sensor_msgs::msg::PointCloud2 cloud;
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;
  serialization.deserialize_message(&serialized, &cloud);
  return cloud;
}

livox_ros_driver2::msg::CustomMsg python_reference(const sensor_msgs::msg::PointCloud2 & source)
{
  uint32_t x_offset = 0;
  uint32_t y_offset = 0;
  uint32_t z_offset = 0;
  for (const auto & field : source.fields) {
    if (field.name == "x") x_offset = field.offset;
    if (field.name == "y") y_offset = field.offset;
    if (field.name == "z") z_offset = field.offset;
  }
  livox_ros_driver2::msg::CustomMsg expected;
  expected.header = source.header;
  expected.header.frame_id = "livox_frame";
  expected.timebase = static_cast<uint64_t>(source.header.stamp.sec) * 1000000000ULL + source.header.stamp.nanosec;
  expected.lidar_id = 0;
  for (size_t index = 0; index < static_cast<size_t>(source.width) * source.height; ++index) {
    const uint8_t * point = source.data.data() + index * source.point_step;
    float x;
    float y;
    float z;
    std::memcpy(&x, point + x_offset, sizeof(x));
    std::memcpy(&y, point + y_offset, sizeof(y));
    std::memcpy(&z, point + z_offset, sizeof(z));
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
    livox_ros_driver2::msg::CustomPoint output;
    output.x = x;
    output.y = y;
    output.z = z;
    output.reflectivity = 128;
    output.tag = 0x10;
    output.line = static_cast<uint8_t>(expected.points.size() % 4);
    expected.points.push_back(output);
  }
  expected.point_num = expected.points.size();
  for (size_t index = 0; index < expected.points.size(); ++index) {
    expected.points[index].offset_time = expected.points.size() < 2 ? 0 :
      static_cast<uint32_t>(index * 100000000ULL / (expected.points.size() - 1));
  }
  return expected;
}

std::vector<uint8_t> serialize(const livox_ros_driver2::msg::CustomMsg & message)
{
  rclcpp::Serialization<livox_ros_driver2::msg::CustomMsg> serialization;
  rclcpp::SerializedMessage serialized;
  serialization.serialize_message(&message, &serialized);
  const auto & raw = serialized.get_rcl_serialized_message();
  return {raw.buffer, raw.buffer + raw.buffer_length};
}

sensor_msgs::msg::PointCloud2 minimal_cloud(size_t count)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp.sec = 17;
  cloud.header.stamp.nanosec = 23;
  cloud.header.frame_id = "source";
  cloud.width = count;
  cloud.height = 1;
  cloud.point_step = 12;
  cloud.row_step = count * cloud.point_step;
  cloud.fields.resize(3);
  cloud.fields[0].name = "x";
  cloud.fields[0].offset = 0;
  cloud.fields[0].datatype = 7;
  cloud.fields[0].count = 1;
  cloud.fields[1].name = "y";
  cloud.fields[1].offset = 4;
  cloud.fields[1].datatype = 7;
  cloud.fields[1].count = 1;
  cloud.fields[2].name = "z";
  cloud.fields[2].offset = 8;
  cloud.fields[2].datatype = 7;
  cloud.fields[2].count = 1;
  cloud.data.resize(cloud.row_step);
  for (size_t index = 0; index < count; ++index) {
    const float value = static_cast<float>(index + 1);
    std::memcpy(cloud.data.data() + index * 12, &value, sizeof(value));
    std::memcpy(cloud.data.data() + index * 12 + 4, &value, sizeof(value));
    std::memcpy(cloud.data.data() + index * 12 + 8, &value, sizeof(value));
  }
  return cloud;
}
}  // namespace

TEST(Converter, CapturedCloudMatchesPythonReferenceAndPreservesPayload)
{
  const auto source = read_captured_cloud();
  ASSERT_EQ(source.width, 720u);
  ASSERT_EQ(source.height, 16u);
  ASSERT_EQ(source.point_step, 32u);
  ASSERT_EQ(source.row_step, 23040u);
  ASSERT_EQ(source.data.size(), 368640u);
  sensor_msgs::msg::PointCloud2 adapted;
  livox_ros_driver2::msg::CustomMsg converted;
  ASSERT_TRUE(soarm_native_livox_converter::convert(source, adapted, converted));
  const auto expected = python_reference(source);
  EXPECT_EQ(adapted.header.stamp, source.header.stamp);
  EXPECT_EQ(adapted.header.frame_id, "livox_frame");
  EXPECT_EQ(adapted.height, source.height);
  EXPECT_EQ(adapted.width, source.width);
  EXPECT_EQ(adapted.fields, source.fields);
  EXPECT_EQ(adapted.is_bigendian, source.is_bigendian);
  EXPECT_EQ(adapted.point_step, source.point_step);
  EXPECT_EQ(adapted.row_step, source.row_step);
  EXPECT_EQ(adapted.data, source.data);
  EXPECT_EQ(serialize(converted), serialize(expected));
}

TEST(Converter, FiltersNaNsAndMatchesReference)
{
  auto source = minimal_cloud(3);
  const float nan = std::nanf("");
  std::memcpy(source.data.data() + 12 + 4, &nan, sizeof(nan));
  sensor_msgs::msg::PointCloud2 adapted;
  livox_ros_driver2::msg::CustomMsg converted;
  ASSERT_TRUE(soarm_native_livox_converter::convert(source, adapted, converted));
  EXPECT_EQ(serialize(converted), serialize(python_reference(source)));
  ASSERT_EQ(converted.points.size(), 2u);
  EXPECT_EQ(converted.points[0].offset_time, 0u);
  EXPECT_EQ(converted.points[1].offset_time, 100000000u);
}

TEST(Converter, ZeroAndOnePointCloudsAreValid)
{
  for (size_t count : {0u, 1u}) {
    auto source = minimal_cloud(count);
    sensor_msgs::msg::PointCloud2 adapted;
    livox_ros_driver2::msg::CustomMsg converted;
    ASSERT_TRUE(soarm_native_livox_converter::convert(source, adapted, converted));
    EXPECT_EQ(converted.points.size(), count);
    if (count == 1u) EXPECT_EQ(converted.points[0].offset_time, 0u);
  }
}

TEST(Converter, RejectsMalformedLayoutsWithoutOutputMutation)
{
  auto source = minimal_cloud(2);
  sensor_msgs::msg::PointCloud2 adapted = minimal_cloud(1);
  livox_ros_driver2::msg::CustomMsg converted;
  converted.point_num = 42;
  source.row_step -= 1;
  EXPECT_FALSE(soarm_native_livox_converter::convert(source, adapted, converted));
  EXPECT_EQ(converted.point_num, 42u);
  source = minimal_cloud(2);
  source.is_bigendian = true;
  EXPECT_FALSE(soarm_native_livox_converter::convert(source, adapted, converted));
  source = minimal_cloud(2);
  source.fields[0].datatype = 2;
  EXPECT_FALSE(soarm_native_livox_converter::convert(source, adapted, converted));
  source = minimal_cloud(2);
  source.fields[1].count = 2;
  EXPECT_FALSE(soarm_native_livox_converter::convert(source, adapted, converted));
  source = minimal_cloud(2);
  source.fields[2].offset = source.point_step - 3;
  EXPECT_FALSE(soarm_native_livox_converter::convert(source, adapted, converted));
  source = minimal_cloud(2);
  source.data.pop_back();
  EXPECT_FALSE(soarm_native_livox_converter::convert(source, adapted, converted));
}

TEST(Converter, RejectsPointCountOverflowWithoutAllocatingPayload)
{
  auto source = minimal_cloud(0);
  source.width = std::numeric_limits<uint32_t>::max() / 4;
  source.height = 5;
  source.point_step = 4;
  source.row_step = 0;
  source.data.clear();
  sensor_msgs::msg::PointCloud2 adapted;
  livox_ros_driver2::msg::CustomMsg converted;
  EXPECT_FALSE(soarm_native_livox_converter::convert(source, adapted, converted));
}
