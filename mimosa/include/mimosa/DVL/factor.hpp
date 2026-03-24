// Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace mimosa
{
namespace dvl
{
/**
 * DVL velocity factor with lever arm compensation.
 *
 * Measurement model:
 *   v_D = R_D_B * (R_B_W * v_W + (omega_B - b_g) x t_B_D)
 *
 * where:
 *   v_D:     measured velocity in DVL sensor frame
 *   R_D_B:   rotation from body to DVL frame (from T_B_D extrinsics)
 *   R_B_W:   rotation from world to body (from estimated pose)
 *   v_W:     linear velocity in world frame (estimated)
 *   omega_B: angular velocity from IMU in body frame
 *   b_g:     gyroscope bias (estimated)
 *   t_B_D:   lever arm from body origin to DVL sensor in body frame
 *
 * Keys: X(pose), V(velocity), B(imu_bias)
 */
class DVLFactor
: public gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Vector3, gtsam::imuBias::ConstantBias>
{
  gtsam::Vector3 measured_vel_sensor_;   // measured velocity in DVL sensor frame
  gtsam::Pose3 pose_D_B_;               // extrinsic: DVL pose in body frame (T_B_D)
  gtsam::Vector3 angular_velocity_B_;   // angular velocity from IMU in body frame

public:
  typedef NoiseModelFactor3<gtsam::Pose3, gtsam::Vector3, gtsam::imuBias::ConstantBias> Base;

  DVLFactor(
    const gtsam::Vector3 & measured_vel_sensor, const gtsam::Pose3 & pose_D_B,
    const gtsam::Vector3 & angular_velocity_B, const gtsam::Key k0, const gtsam::Key k1,
    const gtsam::Key k2, const gtsam::SharedNoiseModel & noise_model)
  : Base(noise_model, k0, k1, k2),
    measured_vel_sensor_(measured_vel_sensor),
    pose_D_B_(pose_D_B),
    angular_velocity_B_(angular_velocity_B)
  {
  }

  virtual ~DVLFactor() {}

  gtsam::NonlinearFactor::shared_ptr clone() const override
  {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new DVLFactor(*this)));
  }

  gtsam::Vector evaluateError(
    const gtsam::Pose3 & pose_B_W, const gtsam::Vector3 & linear_velocity_W,
    const gtsam::imuBias::ConstantBias & imu_bias_B,
    gtsam::Matrix * H0 = nullptr, gtsam::Matrix * H1 = nullptr,
    gtsam::Matrix * H2 = nullptr) const override
  {
    const gtsam::Rot3 rot_D_B = pose_D_B_.rotation();     // R from {D} to {B}
    const gtsam::Point3 t_B_D = pose_D_B_.translation();  // lever arm: body to DVL in {B}
    const gtsam::Rot3 rot_B_W = pose_B_W.rotation();      // R from {B} to {W}

    // Lever arm contribution: (omega_B - b_g) x t_B_D
    const gtsam::Vector3 omega_corrected = angular_velocity_B_ - imu_bias_B.gyroscope();
    const gtsam::Vector3 lever_arm_vel_B = omega_corrected.cross(t_B_D);

    // Full velocity in DVL frame:
    // v_D = R_D_B^T * (R_B_W^T * v_W + lever_arm_vel_B)
    const gtsam::Vector3 vel_body = rot_B_W.transpose() * linear_velocity_W + lever_arm_vel_B;
    const gtsam::Vector3 vel_sensor_estimate = rot_D_B.transpose() * vel_body;

    const gtsam::Vector3 residual = vel_sensor_estimate - measured_vel_sensor_;

    // df/d(pose_B_W) — only rotation part matters (translation has no effect)
    if (H0) {
      H0->resize(3, 6);
      H0->leftCols(3) = rot_D_B.transpose() * rot_B_W.transpose() *
                         gtsam::skewSymmetric(linear_velocity_W) * rot_B_W.matrix();
      H0->rightCols(3) = gtsam::Matrix33::Zero();
    }

    // df/d(velocity_W)
    if (H1) {
      H1->resize(3, 3);
      *H1 = rot_D_B.transpose() * rot_B_W.transpose();
    }

    // df/d(imu_bias) = [d/d(acc_bias) | d/d(gyro_bias)]
    // lever_arm_vel = (omega - b_g) x t_B_D = skew(omega - b_g) * t_B_D
    // d/d(b_g): skew(a)*b => d/da = -skew(b), chain rule with -b_g gives skew(t_B_D)
    if (H2) {
      H2->resize(3, 6);
      H2->leftCols(3) = gtsam::Matrix33::Zero();
      H2->rightCols(3) = rot_D_B.transpose() * gtsam::skewSymmetric(t_B_D);
    }

    return residual;
  }
};
}  // namespace dvl
}  // namespace mimosa
