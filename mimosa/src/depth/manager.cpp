// Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#include "mimosa/depth/manager.hpp"

#include "mimosa/state.hpp"

namespace mimosa
{
namespace depth
{
Manager::Manager(
  const std::string & config_path, ri::NodeHandle & nh, mimosa::imu::Manager::SharedPtr imu_manager,
  mimosa::graph::Manager::SharedPtr graph_manager)
: SensorManagerBase<ManagerConfig, ri::SensorMsgsFluidPressure>(
    config::checkValid(config::fromYamlFile<ManagerConfig>(config_path)), nh, imu_manager,
    graph_manager, "depth")
{
  subscribeIfEnabled();
}

void Manager::callback(const ri::ConstSharedPtr<ri::SensorMsgsFluidPressure> & msg)
{
  if (!passesCommonValidations(msg)) {
    return;
  }
  corrected_ts_ = header_ts_ + config_.base.ts_offset;

  // Convert pressure (Pa) to signed depth (m). World z is up, so submerged
  // sensors have negative z. The matching residual lives in factor.hpp.
  const double depth_below_surface =
    (msg->fluid_pressure - config_.surface_pressure) /
    (config_.fluid_density * config_.gravity_magnitude);
  const double raw_measured_z = -depth_below_surface;

  // Capture an offset on the first message so the first depth factor has zero
  // residual against the init pose (world-z = 0). After that, every factor
  // measures delta-z relative to where the robot started.
  if (!depth_offset_initialized_) {
    depth_offset_ = raw_measured_z;
    depth_offset_initialized_ = true;
    logger_->info(
      "Captured depth offset from first pressure message: {} m (pressure: {} Pa)",
      depth_offset_, msg->fluid_pressure);
  }
  const double measured_z = raw_measured_z - depth_offset_;

  auto noise_model = gtsam::noiseModel::Isotropic::Sigma(1, config_.sigma_depth_m);

  gtsam::NonlinearFactorGraph new_factors;
  auto factor =
    std::make_shared<DepthFactor>(measured_z, config_.base.T_B_S, X(0), noise_model);
  new_factors.add(factor);

  logger_->debug("Declaring depth factor (ts: {} measured_z: {})", corrected_ts_, measured_z);
  graph::Manager::DeclarationResult dr = graph_manager_->declare(
    corrected_ts_, new_key_, config_.base.use_to_init, new_factors);

  if (!handleDeclarationResult(dr)) {
    return;
  }
  initialized_ = true;
  logger_->debug("Finished processing depth measurement. Assigned key: {}", gdkf(new_key_));
}

void declare_config(ManagerConfig & config)
{
  using namespace config;
  name("Depth Manager Config");

  // Declare common sensor manager config fields
  declare_sensor_manager_config_base(config.base, "depth");

  // Declare depth-specific fields
  {
    NameSpace ns("depth");
    {
      NameSpace ns("manager");
      field(config.sigma_depth_m, "sigma_depth_m", "noise sigma for depth measurement [m]");
      field(config.fluid_density, "fluid_density", "[kg/m^3]");
      field(config.surface_pressure, "surface_pressure", "[Pa]");
      field(config.gravity_magnitude, "gravity_magnitude", "[m/s^2]");
    }
  }

  check(config.sigma_depth_m, GT, 0.0f, "sigma_depth_m");
  check(config.fluid_density, GT, 0.0f, "fluid_density");
  check(config.gravity_magnitude, GT, 0.0f, "gravity_magnitude");
}

}  // namespace depth
}  // namespace mimosa
