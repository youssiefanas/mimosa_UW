// Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

// Bridge: interfaces/msg/IMU (Nortek Nucleus driver) -> sensor_msgs/msg/Imu.
//
//   Subscribes:  ~/imu_in   (interfaces/msg/IMU)    default /nucleus_node/imu_packets
//   Publishes:   ~/imu_out  (sensor_msgs/msg/Imu)   default /nortek/imu
//
// Field mapping:
//   header.stamp        <- system_timestamp
//   header.frame_id     <- ~frame_id parameter (default "nortek_imu_link")
//   linear_acceleration <- accelerometer_x/y/z   (assumed m/s^2)
//   angular_velocity    <- gyro_x/y/z            (assumed rad/s)
//   orientation         <- not provided; orientation_covariance[0] = -1 per REP-145
//
// Messages with is_valid == false are dropped.

#include <interfaces/msg/imu.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace
{
class NortekImuBridge : public rclcpp::Node
{
public:
  NortekImuBridge() : rclcpp::Node("nortek_imu_bridge")
  {
    frame_id_ = this->declare_parameter<std::string>("frame_id", "nortek_imu_link");

    pub_ = this->create_publisher<sensor_msgs::msg::Imu>("~/imu_out", 50);
    sub_ = this->create_subscription<interfaces::msg::IMU>(
      "~/imu_in", 50,
      [this](const interfaces::msg::IMU::ConstSharedPtr & msg) { this->onMsg(msg); });

    RCLCPP_INFO(
      this->get_logger(),
      "nortek_imu_bridge ready: subscribing ~/imu_in -> publishing ~/imu_out (frame_id=%s)",
      frame_id_.c_str());
  }

private:
  void onMsg(const interfaces::msg::IMU::ConstSharedPtr & msg)
  {
    if (!msg->is_valid) {
      return;
    }

    sensor_msgs::msg::Imu out;
    out.header.stamp = msg->system_timestamp;
    out.header.frame_id = frame_id_;

    out.linear_acceleration.x = msg->accelerometer_x;
    out.linear_acceleration.y = msg->accelerometer_y;
    out.linear_acceleration.z = msg->accelerometer_z;

    out.angular_velocity.x = msg->gyro_x;
    out.angular_velocity.y = msg->gyro_y;
    out.angular_velocity.z = msg->gyro_z;

    // Orientation not provided by the raw IMU; signal "unknown" per REP-145.
    out.orientation_covariance[0] = -1.0;

    pub_->publish(out);
  }

  std::string frame_id_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::Subscription<interfaces::msg::IMU>::SharedPtr sub_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NortekImuBridge>());
  rclcpp::shutdown();
  return 0;
}
