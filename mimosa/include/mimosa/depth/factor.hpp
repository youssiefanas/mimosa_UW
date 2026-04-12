// Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace mimosa
{
namespace depth
{
/**
 * Unary depth-prior factor for a pressure-derived depth measurement.
 *
 * Convention:
 *   - World frame is z-up (mimosa default).
 *   - measured_depth is the SIGNED Z of the sensor in the world frame
 *     (i.e. negative when below the surface). The manager is responsible
 *     for converting raw pressure to this signed value.
 *
 * Measurement model (lever-arm compensated):
 *   z_S = (T_W_B * t_B_S).z()
 *   residual = z_S - measured_depth
 *
 * Key: X(pose_W_B)
 */
class DepthFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3>
{
  double measured_depth_;     // signed world-frame Z of the sensor [m]
  gtsam::Point3 t_B_S_;       // lever arm: sensor position in body frame

public:
  typedef gtsam::NoiseModelFactor1<gtsam::Pose3> Base;

  DepthFactor(
    double measured_depth, const gtsam::Pose3 & T_B_S, const gtsam::Key key,
    const gtsam::SharedNoiseModel & noise_model)
  : Base(noise_model, key), measured_depth_(measured_depth), t_B_S_(T_B_S.translation())
  {
  }

  ~DepthFactor() override = default;

  gtsam::NonlinearFactor::shared_ptr clone() const override
  {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new DepthFactor(*this)));
  }

  gtsam::Vector evaluateError(
    const gtsam::Pose3 & pose_W_B, gtsam::Matrix * H = nullptr) const override
  {
    // p_W_S = T_W_B * t_B_S. Hpoint not needed (lever arm is constant).
    gtsam::Matrix Hfull;  // 3x6 jacobian of p_W_S w.r.t. pose_W_B tangent
    const gtsam::Point3 p_W_S = pose_W_B.transformFrom(t_B_S_, H ? &Hfull : nullptr);

    if (H) {
      // Select only the z row -> 1x6 jacobian of the scalar depth residual.
      *H = Hfull.row(2);
    }

    return (gtsam::Vector(1) << (p_W_S.z() - measured_depth_)).finished();
  }
};

}  // namespace depth
}  // namespace mimosa
