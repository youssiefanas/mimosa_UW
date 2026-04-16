// Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#include "mimosa/DVL/manager.hpp"

namespace mimosa
{
namespace dvl
{

// ============================================================================
// WaterlinkedManager
// ============================================================================
WaterlinkedManager::WaterlinkedManager(
  const std::string & config_path, ri::NodeHandle & nh, mimosa::imu::Manager::SharedPtr imu_manager,
  mimosa::graph::Manager::SharedPtr graph_manager)
: SensorManagerBase<ManagerConfig, ri::WaterlinkedDVL>(
    config::checkValid(config::fromYamlFile<ManagerConfig>(config_path)), nh, imu_manager,
    graph_manager, "dvl")
{
  subscribeIfEnabled();
}

void WaterlinkedManager::callback(const ri::ConstSharedPtr<ri::WaterlinkedDVL> & msg)
{
  if (!passesCommonValidations(msg)) {
    return;
  }
  corrected_ts_ = header_ts_ + config_.base.ts_offset;

  if (!msg->velocity_valid) {
    logger_->debug("DVL velocity not valid. Skipping.");
    return;
  }

  // Check FOM
  if (msg->fom > config_.max_fom) {
    logger_->debug("FOM {} exceeds max_fom {}. Skipping.", msg->fom, config_.max_fom);
    return;
  }

  const gtsam::Vector3 vel_sensor(msg->velocity.x, msg->velocity.y, msg->velocity.z);

  // Check velocity magnitude
  if (vel_sensor.norm() > config_.max_velocity) {
    logger_->warn("Velocity magnitude {} exceeds max_velocity {}. Skipping.",
      vel_sensor.norm(), config_.max_velocity);
    return;
  }

  // Waterlinked provides a single FOM for all axes
  const double sigma = msg->fom * config_.fom_scale;
  const gtsam::Vector3 noise_sigmas(sigma, sigma, sigma);

  processVelocity(vel_sensor, noise_sigmas, corrected_ts_);
}

void WaterlinkedManager::processVelocity(
  const gtsam::Vector3 & vel_sensor, const gtsam::Vector3 & noise_sigmas, double timestamp)
{
  static bool first = true;
  if (first) {
    first = false;
    broadcastStaticTransform(timestamp);
  }

  // Get angular velocity from IMU
  V3D angular_velocity_mean = V3D::Zero();
  ImuBuffer imu_measurements;
  try {
    // Query a window ending slightly before the DVL timestamp to ensure IMU data is available.
    // DVL timestamps are typically ~1-2ms ahead of the latest IMU sample in the buffer.
    constexpr double kDelay = 0.0;   // no offset
    constexpr double kEps = 3e-3;   // 3ms half-window
    imu_manager_->getInterpolatedMeasurements(
      timestamp - kDelay - kEps, timestamp - kDelay + kEps, imu_measurements);

    size_t n = 0;
    for (auto it = imu_measurements.begin(); it != imu_measurements.end(); ++it) {
      angular_velocity_mean += it->second.tail<3>();
      n++;
    }
    if (n > 0) {
      angular_velocity_mean /= static_cast<double>(n);
    }
  } catch (const std::exception & e) {
    logger_->warn("IMU data not available at DVL timestamp, using zero angular velocity: {}", e.what());
  }

  // Create noise model with per-axis sigmas
  auto noise_model = gtsam::noiseModel::Diagonal::Sigmas(noise_sigmas);

  // Create factor with lever arm compensation
  gtsam::NonlinearFactorGraph new_factors;
  auto factor = std::make_shared<DVLFactor>(
    vel_sensor, config_.base.T_B_S, angular_velocity_mean, X(0), V(0), B(0), noise_model);
  new_factors.add(factor);

  // Body-frame velocity hint for initialization warm-start. Invert the DVL
  // measurement model (lever-arm compensated):
  //   v_S = R_B_S^T * (v_B + omega_B x t_B_S)
  //   => v_B = R_B_S * v_S - omega_B x t_B_S
  // t_B_S is already expressed in the body frame (it's the translation of T_B_S).
  const gtsam::Rot3 & R_B_S = config_.base.T_B_S.rotation();
  const V3D & t_B_S = config_.base.T_B_S.translation();
  const V3D v_B_hint = R_B_S * vel_sensor - angular_velocity_mean.cross(t_B_S);

  logger_->debug("Declaring DVL factor (ts: {})", timestamp);
  logger_->debug(
  "declare() — ts: {:.6f}, key: {}, provisional key: {}, use_to_init: {}, one_step_factors: {}",
  corrected_ts_, new_key_, gdkf(new_key_), config_.base.use_to_init, new_factors.size());
  graph::Manager::DeclarationResult dr = graph_manager_->declare(
    timestamp, new_key_, config_.base.use_to_init, new_factors, v_B_hint);

  if (!handleDeclarationResult(dr)) {
    return;
  }
  initialized_ = true;
  logger_->debug("Finished processing DVL measurement. Assigned key: {}", gdkf(new_key_));
}

// ============================================================================
// NortekManager
// ============================================================================
NortekManager::NortekManager(
  const std::string & config_path, ri::NodeHandle & nh, mimosa::imu::Manager::SharedPtr imu_manager,
  mimosa::graph::Manager::SharedPtr graph_manager)
: SensorManagerBase<ManagerConfig, ri::NortekBottomTrack>(
    config::checkValid(config::fromYamlFile<ManagerConfig>(config_path)), nh, imu_manager,
    graph_manager, "dvl")
{
  subscribeIfEnabled();
}

void NortekManager::callback(const ri::ConstSharedPtr<ri::NortekBottomTrack> & msg)
{
  if (!passesCommonValidations(msg)) {
    return;
  }
  corrected_ts_ = header_ts_ + config_.base.ts_offset;

  // Check per-axis validity
  if (!msg->x_velocity_valid || !msg->y_velocity_valid || !msg->z_velocity_valid) {
    logger_->debug("Nortek DVL velocity not fully valid (x:{}, y:{}, z:{}). Skipping.",
      msg->x_velocity_valid, msg->y_velocity_valid, msg->z_velocity_valid);
    return;
  }

  // Check per-axis FOM validity
  if (!msg->x_fom_valid || !msg->y_fom_valid || !msg->z_fom_valid) {
    logger_->debug("Nortek DVL FOM not fully valid. Skipping.");
    return;
  }

  // Check per-axis FOM thresholds
  if (msg->fom_x > config_.max_fom || msg->fom_y > config_.max_fom ||
      msg->fom_z > config_.max_fom) {
    logger_->debug("Nortek FOM exceeds threshold (x:{}, y:{}, z:{}). Skipping.",
      msg->fom_x, msg->fom_y, msg->fom_z);
    return;
  }

  const gtsam::Vector3 vel_sensor(msg->velocity_x, msg->velocity_y, msg->velocity_z);

  // Check velocity magnitude
  if (vel_sensor.norm() > config_.max_velocity) {
    logger_->warn("Velocity magnitude {} exceeds max_velocity {}. Skipping.",
      vel_sensor.norm(), config_.max_velocity);
    return;
  }

  // Per-axis noise from FOM
  const gtsam::Vector3 noise_sigmas(
    msg->fom_x * config_.fom_scale,
    msg->fom_y * config_.fom_scale,
    msg->fom_z * config_.fom_scale);

  processVelocity(vel_sensor, noise_sigmas, corrected_ts_);
}

void NortekManager::processVelocity(
  const gtsam::Vector3 & vel_sensor, const gtsam::Vector3 & noise_sigmas, double timestamp)
{
  static bool first = true;
  if (first) {
    first = false;
    broadcastStaticTransform(timestamp);
  }

  // Get angular velocity from IMU
  V3D angular_velocity_mean = V3D::Zero();
  ImuBuffer imu_measurements;
  try {
    // Query a window ending slightly before the DVL timestamp to ensure IMU data is available.
    // DVL timestamps are typically ~1-2ms ahead of the latest IMU sample in the buffer.
    constexpr double kDelay = 0.0;   // no offset
    constexpr double kEps = 3e-3;   // 3ms half-window
    imu_manager_->getInterpolatedMeasurements(
      timestamp - kDelay - kEps, timestamp - kDelay + kEps, imu_measurements);

    size_t n = 0;
    for (auto it = imu_measurements.begin(); it != imu_measurements.end(); ++it) {
      angular_velocity_mean += it->second.tail<3>();
      n++;
    }
    if (n > 0) {
      angular_velocity_mean /= static_cast<double>(n);
    }
  } catch (const std::exception & e) {
    logger_->warn("IMU data not available at DVL timestamp, using zero angular velocity: {}", e.what());
  }

  // Create noise model with per-axis sigmas
  auto noise_model = gtsam::noiseModel::Diagonal::Sigmas(noise_sigmas);

  // Create factor with lever arm compensation
  gtsam::NonlinearFactorGraph new_factors;
  auto factor = std::make_shared<DVLFactor>(
    vel_sensor, config_.base.T_B_S, angular_velocity_mean, X(0), V(0), B(0), noise_model);
  new_factors.add(factor);

  // Body-frame velocity hint for initialization warm-start. Invert the DVL
  // measurement model (lever-arm compensated):
  //   v_S = R_B_S^T * (v_B + omega_B x t_B_S)
  //   => v_B = R_B_S * v_S - omega_B x t_B_S
  // t_B_S is already expressed in the body frame (it's the translation of T_B_S).
  const gtsam::Rot3 & R_B_S = config_.base.T_B_S.rotation();
  const V3D & t_B_S = config_.base.T_B_S.translation();
  const V3D v_B_hint = R_B_S * vel_sensor - angular_velocity_mean.cross(t_B_S);

  logger_->debug("Declaring DVL factor (ts: {})", timestamp);
  graph::Manager::DeclarationResult dr = graph_manager_->declare(
    timestamp, new_key_, config_.base.use_to_init, new_factors, v_B_hint);

  if (!handleDeclarationResult(dr)) {
    return;
  }
  initialized_ = true;
  logger_->debug("Finished processing DVL measurement. Assigned key: {}", gdkf(new_key_));
}

// ============================================================================
// Unified Manager
// ============================================================================
Manager::Manager(
  const std::string & config_path, ri::NodeHandle & nh, mimosa::imu::Manager::SharedPtr imu_manager,
  mimosa::graph::Manager::SharedPtr graph_manager)
{
  auto config = config::fromYamlFile<ManagerConfig>(config_path);
  type_ = static_cast<DVLType>(config.dvl_type);
  DVLType type = type_;

  switch (type) {
    case DVLType::Waterlinked:
      waterlinked_manager_ =
        std::make_unique<WaterlinkedManager>(config_path, nh, imu_manager, graph_manager);
      break;
    case DVLType::Nortek:
      nortek_manager_ =
        std::make_unique<NortekManager>(config_path, nh, imu_manager, graph_manager);
      break;
  }
}

// ============================================================================
// Config declaration
// ============================================================================
void declare_config(ManagerConfig & config)
{
  using namespace config;
  name("DVL Manager Config");

  // Declare common sensor manager config fields
  declare_sensor_manager_config_base(config.base, "dvl");

  // Declare DVL-specific fields
  {
    NameSpace ns("dvl");
    {
      NameSpace ns("manager");
      field(config.dvl_type, "dvl_type", "0=Waterlinked, 1=Nortek");
      field(config.fom_scale, "fom_scale", "noise sigma = fom * fom_scale");
      field(config.max_fom, "max_fom", "m/s");
      field(config.max_velocity, "max_velocity", "m/s");
    }
  }
}

}  // namespace dvl
}  // namespace mimosa
