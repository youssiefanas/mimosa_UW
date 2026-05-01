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
// Frame convention:
//   The Nortek Nucleus publishes IMU samples in a z-down (NED-style) body
//   frame: a stationary IMU reads acc_z = -g. mimosa's IMU manager assumes a
//   z-up body (acc_z = +g at rest) — see imu/manager.cpp:57,200. With
//   `rotate_to_zup` enabled (default), this bridge applies a 180° rotation
//   about the x-axis to both linear_acceleration and angular_velocity, which
//   negates the y and z components. The published frame is therefore z-up.
//   Any other sensor that is rigidly co-located with the Nortek (DVL, pressure)
//   must have its T_B_S in params.yaml encode the same 180°-about-x rotation:
//   `T_B_S: [tx, -ty_old, -tz_old, 1.0, 0.0, 0.0, 0.0]` (qx=1, qw=0).
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
    rotate_to_zup_ = this->declare_parameter<bool>("rotate_to_zup", true);

    pub_ = this->create_publisher<sensor_msgs::msg::Imu>("~/imu_out", 50);
    sub_ = this->create_subscription<interfaces::msg::IMU>(
      "~/imu_in", 50,
      [this](const interfaces::msg::IMU::ConstSharedPtr & msg) { this->onMsg(msg); });

    RCLCPP_INFO(
      this->get_logger(),
      "nortek_imu_bridge ready: subscribing ~/imu_in -> publishing ~/imu_out "
      "(frame_id=%s, rotate_to_zup=%s)",
      frame_id_.c_str(), rotate_to_zup_ ? "true" : "false");
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

    // 180°-about-x rotation flips y and z signs of any body-frame 3-vector;
    // x is unchanged. Applied to both linear_acceleration and angular_velocity.
    const double s = rotate_to_zup_ ? -1.0 : 1.0;

    out.linear_acceleration.x = msg->accelerometer_x;
    out.linear_acceleration.y = s * msg->accelerometer_y;
    out.linear_acceleration.z = s * msg->accelerometer_z;

    out.angular_velocity.x = msg->gyro_x;
    out.angular_velocity.y = s * msg->gyro_y;
    out.angular_velocity.z = s * msg->gyro_z;

    // Orientation not provided by the raw IMU; signal "unknown" per REP-145.
    out.orientation_covariance[0] = -1.0;

    pub_->publish(out);
  }

  std::string frame_id_;
  bool rotate_to_zup_{true};
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
