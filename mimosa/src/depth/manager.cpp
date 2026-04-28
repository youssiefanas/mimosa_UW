// Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#include "mimosa/depth/manager.hpp"

#include <numeric>

#include "mimosa/state.hpp"

namespace mimosa
{
namespace depth
{

// ============================================================================
// DepthProcessor
// ============================================================================
DepthProcessor::DepthProcessor(const ManagerConfig & config, spdlog::logger * logger)
: config_(config), logger_(logger)
{
}

std::optional<DepthProcessor::ProcessResult> DepthProcessor::process(double pressure_pa)
{
  // Optional startup zero-offset calibration. While it's running we still
  // accumulate samples but skip declaring a factor; this assumes the vehicle
  // is at the surface and stationary during these first messages.
  if (config_.n_calibration_samples > 0 && !pressure_offset_pa_.has_value()) {
    calibration_samples_.push_back(pressure_pa);
    if (
      static_cast<int>(calibration_samples_.size()) >= config_.n_calibration_samples) {
      const double mean =
        std::accumulate(calibration_samples_.begin(), calibration_samples_.end(), 0.0) /
        static_cast<double>(calibration_samples_.size());
      pressure_offset_pa_ = mean;
      if (logger_) {
        logger_->info(
          "Depth pressure offset calibrated to {:.1f} Pa (~{:.4f} bar) over {} samples", mean,
          mean / 100000.0, calibration_samples_.size());
      }
      calibration_samples_.clear();
    }
    return std::nullopt;
  }

  const double offset_pa = pressure_offset_pa_.value_or(0.0);

  // Convert pressure (Pa) to signed depth (m). World z is up, so submerged
  // sensors have negative z. The matching residual lives in factor.hpp.
  const double depth_below_surface =
    (pressure_pa - config_.surface_pressure - offset_pa) /
    (config_.fluid_density * config_.gravity_magnitude);
  const double measured_z = -depth_below_surface;

  {
    std::lock_guard<std::mutex> lk(z_mutex_);
    latest_measured_z_ = measured_z;
  }

  auto noise_model = gtsam::noiseModel::Isotropic::Sigma(1, config_.sigma_depth_m);
  gtsam::NonlinearFactorGraph factors;
  factors.add(std::make_shared<DepthFactor>(measured_z, config_.base.T_B_S, X(0), noise_model));

  return ProcessResult{std::move(factors), measured_z};
}

std::optional<double> DepthProcessor::getInitZHint()
{
  std::lock_guard<std::mutex> lk(z_mutex_);
  return latest_measured_z_;
}

// ============================================================================
// FluidPressureManager
// ============================================================================
FluidPressureManager::FluidPressureManager(
  const std::string & config_path, ri::NodeHandle & nh, mimosa::imu::Manager::SharedPtr imu_manager,
  mimosa::graph::Manager::SharedPtr graph_manager)
: SensorManagerBase<ManagerConfig, ri::SensorMsgsFluidPressure>(
    config::checkValid(config::fromYamlFile<ManagerConfig>(config_path)), nh, imu_manager,
    graph_manager, "depth"),
  processor_(config_, logger_.get())
{
  subscribeIfEnabled();
}

void FluidPressureManager::callback(const ri::ConstSharedPtr<ri::SensorMsgsFluidPressure> & msg)
{
  if (!passesCommonValidations(msg)) {
    return;
  }
  corrected_ts_ = header_ts_ + config_.base.ts_offset;

  // FluidPressure here is published in bar (the user's local convention);
  // multiply by 100000 to convert to Pa.
  const double pressure_pa = msg->fluid_pressure * 100000.0;

  auto result = processor_.process(pressure_pa);
  if (!result.has_value()) {
    return;
  }

  logger_->debug(
    "Declaring depth factor (ts: {} measured_z: {})", corrected_ts_, result->measured_z);
  logger_->debug(
    "declare() — ts: {:.6f}, key: {}, provisional key: {}, use_to_init: {}, one_step_factors: {}",
    corrected_ts_, new_key_, gdkf(new_key_), config_.base.use_to_init, result->factors.size());

  graph::Manager::DeclarationResult dr = graph_manager_->declare(
    corrected_ts_, new_key_, config_.base.use_to_init, result->factors, std::nullopt,
    result->measured_z);

  if (!handleDeclarationResult(dr)) {
    return;
  }
  initialized_ = true;
  logger_->debug("Finished processing depth measurement. Assigned key: {}", gdkf(new_key_));
}

// ============================================================================
// NortekBottomTrackManager
// ============================================================================
NortekBottomTrackManager::NortekBottomTrackManager(
  const std::string & config_path, ri::NodeHandle & nh, mimosa::imu::Manager::SharedPtr imu_manager,
  mimosa::graph::Manager::SharedPtr graph_manager)
: SensorManagerBase<ManagerConfig, ri::NortekBottomTrack>(
    config::checkValid(config::fromYamlFile<ManagerConfig>(config_path)), nh, imu_manager,
    graph_manager, "depth"),
  processor_(config_, logger_.get())
{
  subscribeIfEnabled();
}

void NortekBottomTrackManager::callback(const ri::ConstSharedPtr<ri::NortekBottomTrack> & msg)
{
  if (!passesCommonValidations(msg)) {
    return;
  }
  corrected_ts_ = header_ts_ + config_.base.ts_offset;

  // BottomTrack.pressure is reported in bar by the Nortek driver; convert to Pa.
  const double pressure_pa = static_cast<double>(msg->pressure) * 100000.0;

  auto result = processor_.process(pressure_pa);
  if (!result.has_value()) {
    return;
  }

  logger_->debug(
    "Declaring depth factor (ts: {} measured_z: {})", corrected_ts_, result->measured_z);
  logger_->debug(
    "declare() — ts: {:.6f}, key: {}, provisional key: {}, use_to_init: {}, one_step_factors: {}",
    corrected_ts_, new_key_, gdkf(new_key_), config_.base.use_to_init, result->factors.size());

  graph::Manager::DeclarationResult dr = graph_manager_->declare(
    corrected_ts_, new_key_, config_.base.use_to_init, result->factors, std::nullopt,
    result->measured_z);

  if (!handleDeclarationResult(dr)) {
    return;
  }
  initialized_ = true;
  logger_->debug("Finished processing depth measurement. Assigned key: {}", gdkf(new_key_));
}

// ============================================================================
// Unified Manager
// ============================================================================
Manager::Manager(
  const std::string & config_path, ri::NodeHandle & nh, mimosa::imu::Manager::SharedPtr imu_manager,
  mimosa::graph::Manager::SharedPtr graph_manager)
{
  auto config = config::fromYamlFile<ManagerConfig>(config_path);
  type_ = static_cast<DepthSource>(config.depth_source);

  switch (type_) {
    case DepthSource::FluidPressure:
      fluid_pressure_manager_ =
        std::make_unique<FluidPressureManager>(config_path, nh, imu_manager, graph_manager);
      graph_manager->setInitZHintProvider(
        [m = fluid_pressure_manager_.get()]() -> std::optional<double> {
          return m->getInitZHint();
        });
      break;
    case DepthSource::NortekBottomTrack:
      nortek_manager_ =
        std::make_unique<NortekBottomTrackManager>(config_path, nh, imu_manager, graph_manager);
      graph_manager->setInitZHintProvider(
        [m = nortek_manager_.get()]() -> std::optional<double> { return m->getInitZHint(); });
      break;
  }
}

// ============================================================================
// Config declaration
// ============================================================================
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
      field(config.depth_source, "depth_source", "0 = FluidPressure msg, 1 = Nortek BottomTrack");
      field(
        config.n_calibration_samples, "n_calibration_samples",
        "0 disables; otherwise average first N samples as zero offset");
    }
  }

  check(config.sigma_depth_m, GT, 0.0f, "sigma_depth_m");
  check(config.fluid_density, GT, 0.0f, "fluid_density");
  check(config.gravity_magnitude, GT, 0.0f, "gravity_magnitude");
  check(config.n_calibration_samples, GE, 0, "n_calibration_samples");
}

}  // namespace depth
}  // namespace mimosa
