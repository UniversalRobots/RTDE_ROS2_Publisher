// Copyright 2026, Universal Robots A/S
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "ur_rtde_publisher/rtde_converter.hpp"

namespace ur_rtde_publisher
{

geometry_msgs::msg::WrenchStamped RTDEConverter::vector6dToWrenchStamped(const std::array<double, 6>& data,
                                                                         const std::string& frame_id,
                                                                         const rclcpp::Time& packet_time)
{
  geometry_msgs::msg::WrenchStamped msg;
  msg.header.stamp = packet_time;
  msg.header.frame_id = frame_id;

  msg.wrench.force.x = data[0];
  msg.wrench.force.y = data[1];
  msg.wrench.force.z = data[2];
  msg.wrench.torque.x = data[3];
  msg.wrench.torque.y = data[4];
  msg.wrench.torque.z = data[5];

  return msg;
}

geometry_msgs::msg::PoseStamped RTDEConverter::vector6dToPoseStamped(const std::array<double, 6>& data,
                                                                     const std::string& frame_id,
                                                                     const rclcpp::Time& packet_time)
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = packet_time;
  msg.header.frame_id = frame_id;

  msg.pose.position.x = data[0];
  msg.pose.position.y = data[1];
  msg.pose.position.z = data[2];

  const double rx = data[3], ry = data[4], rz = data[5];

  const double theta = sqrt(rx * rx + ry * ry + rz * rz);

  tf2::Quaternion q;
  if (theta < 1e-12) {
    q.setValue(0.0, 0.0, 0.0, 1.0);
  } else {
    const double ax = rx / theta;
    const double ay = ry / theta;
    const double az = rz / theta;
    q.setRotation(tf2::Vector3(ax, ay, az), theta);
  }

  msg.pose.orientation = tf2::toMsg(q);
  return msg;
}

geometry_msgs::msg::InertiaStamped RTDEConverter::vector6dToInertiaStamped(const std::array<double, 6>& data,
                                                                           const std::string& frame_id,
                                                                           const rclcpp::Time& packet_time)
{
  geometry_msgs::msg::InertiaStamped msg;
  msg.header.stamp = packet_time;
  msg.header.frame_id = frame_id;

  msg.inertia.ixx = data[0];
  msg.inertia.iyy = data[1];
  msg.inertia.izz = data[2];
  msg.inertia.ixy = data[3];
  msg.inertia.ixz = data[4];
  msg.inertia.iyz = data[5];

  return msg;
}

geometry_msgs::msg::Vector3Stamped RTDEConverter::vector3dToVector3Stamped(const std::array<double, 3>& data,
                                                                           const std::string& frame_id,
                                                                           const rclcpp::Time& packet_time)
{
  geometry_msgs::msg::Vector3Stamped msg;
  msg.header.stamp = packet_time;
  msg.header.frame_id = frame_id;

  msg.vector.x = data[0];
  msg.vector.y = data[1];
  msg.vector.z = data[2];

  return msg;
}

geometry_msgs::msg::PointStamped RTDEConverter::vector3dToPointStamped(const std::array<double, 3>& data,
                                                                       const std::string& frame_id,
                                                                       const rclcpp::Time& packet_time)
{
  geometry_msgs::msg::PointStamped msg;
  msg.header.stamp = packet_time;
  msg.header.frame_id = frame_id;

  msg.point.x = data[0];
  msg.point.y = data[1];
  msg.point.z = data[2];

  return msg;
}

example_interfaces::msg::Float64MultiArray RTDEConverter::vector6dToFloat64MultiArray(const std::array<double, 6>& data)
{
  example_interfaces::msg::Float64MultiArray msg;
  msg.data.reserve(6);

  for (const auto& value : data) {
    msg.data.push_back(value);
  }

  return msg;
}

example_interfaces::msg::ByteMultiArray RTDEConverter::uint32ToByteMultiArray(const uint32_t value)
{
  example_interfaces::msg::ByteMultiArray msg;
  msg.data.reserve(32);

  for (size_t i = 0; i < 32; ++i) {
    uint8_t bit = static_cast<uint8_t>((value >> i) & 0x01);
    msg.data.push_back(bit);
  }

  return msg;
}

example_interfaces::msg::ByteMultiArray RTDEConverter::uint64ToByteMultiArray(const uint64_t value)
{
  example_interfaces::msg::ByteMultiArray msg;
  msg.data.reserve(64);

  for (size_t i = 0; i < 64; ++i) {
    uint8_t bit = static_cast<uint8_t>((value >> i) & 0x01);
    msg.data.push_back(bit);
  }

  return msg;
}

example_interfaces::msg::Int32MultiArray RTDEConverter::vector6intToMsg(const std::array<int32_t, 6>& data)
{
  example_interfaces::msg::Int32MultiArray msg;
  msg.data.reserve(6);

  for (const auto& value : data) {
    msg.data.push_back(static_cast<int32_t>(value));
  }

  return msg;
}

}  // namespace ur_rtde_publisher