#include "soarm_native_livox_converter/converter.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

namespace soarm_native_livox_converter
{
namespace
{
constexpr uint8_t kFloat32 = 7;

bool read_field_offsets(const sensor_msgs::msg::PointCloud2 & source, uint32_t & x, uint32_t & y, uint32_t & z)
{
  bool have_x = false;
  bool have_y = false;
  bool have_z = false;
  for (const auto & field : source.fields) {
    uint32_t * offset = nullptr;
    bool * present = nullptr;
    if (field.name == "x") {
      offset = &x;
      present = &have_x;
    } else if (field.name == "y") {
      offset = &y;
      present = &have_y;
    } else if (field.name == "z") {
      offset = &z;
      present = &have_z;
    } else {
      continue;
    }
    if (*present || field.datatype != kFloat32 || field.count != 1 || field.offset > source.point_step - sizeof(float)) {
      return false;
    }
    *offset = field.offset;
    *present = true;
  }
  return have_x && have_y && have_z;
}

bool valid_layout(const sensor_msgs::msg::PointCloud2 & source, uint32_t & x, uint32_t & y, uint32_t & z)
{
  if (source.is_bigendian || source.point_step < sizeof(float)) return false;
  if (source.width > std::numeric_limits<uint32_t>::max() / source.point_step) return false;
  const uint32_t expected_row_step = source.width * source.point_step;
  if (source.row_step != expected_row_step) return false;
  if (source.row_step == 0) {
    if (!source.data.empty()) return false;
  } else {
    if (source.height > std::numeric_limits<size_t>::max() / source.row_step) return false;
    if (source.data.size() != static_cast<size_t>(source.row_step) * source.height) return false;
  }
  return read_field_offsets(source, x, y, z);
}

float read_float(const uint8_t * data)
{
  float value;
  std::memcpy(&value, data, sizeof(value));
  return value;
}
}  // namespace

bool convert(
  const sensor_msgs::msg::PointCloud2 & source,
  sensor_msgs::msg::PointCloud2 & adapted,
  livox_ros_driver2::msg::CustomMsg & livox)
{
  uint32_t x_offset = 0;
  uint32_t y_offset = 0;
  uint32_t z_offset = 0;
  if (!valid_layout(source, x_offset, y_offset, z_offset)) return false;

  livox_ros_driver2::msg::CustomMsg candidate;
  candidate.header = source.header;
  candidate.header.frame_id = "livox_frame";
  candidate.timebase = static_cast<uint64_t>(source.header.stamp.sec) * 1000000000ULL + source.header.stamp.nanosec;
  candidate.lidar_id = 0;
  const size_t total = static_cast<size_t>(source.width) * source.height;
  candidate.points.reserve(total);
  for (size_t index = 0; index < total; ++index) {
    const uint8_t * point = source.data.data() + index * source.point_step;
    const float x = read_float(point + x_offset);
    const float y = read_float(point + y_offset);
    const float z = read_float(point + z_offset);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
    livox_ros_driver2::msg::CustomPoint output;
    output.x = x;
    output.y = y;
    output.z = z;
    output.reflectivity = 128;
    output.tag = 0x10;
    output.line = static_cast<uint8_t>(candidate.points.size() % 4);
    candidate.points.push_back(output);
  }
  candidate.point_num = static_cast<uint32_t>(candidate.points.size());
  for (size_t index = 0; index < candidate.points.size(); ++index) {
    candidate.points[index].offset_time = candidate.points.size() < 2 ? 0 :
      static_cast<uint32_t>(index * 100000000ULL / (candidate.points.size() - 1));
  }

  adapted = source;
  adapted.header.frame_id = "livox_frame";
  livox = std::move(candidate);
  return true;
}

}  // namespace soarm_native_livox_converter
