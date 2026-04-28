// Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <memory>
#include <mutex>
#include <optional>
#include <vector>

#include "mimosa/depth/factor.hpp"
#include "mimosa/sensor_manager_base.hpp"

namespace mimosa
{
namespace depth
{

enum class DepthSource
{
  FluidPressure = 0,      // sensor_msgs/FluidPressure on its own topic
  NortekBottomTrack = 1,  // pressure field of interfaces/BottomTrack
};

struct ManagerConfig
{
  SensorManagerBaseConfig base;

  // Depth-specific fields
  float sigma_depth_m = 0.05;          // noise sigma for depth measurement [m]
  float fluid_density = 1025.0;        // [kg/m^3] (1000 freshwater, 1025 seawater)
  float surface_pressure = 101325.0;   // baseline pressure subtracted from the reading [Pa]
  float gravity_magnitude = 9.80665;   // [m/s^2]
  int depth_source = 0;                // 0 = FluidPressure msg, 1 = Nortek BottomTrack
  int n_calibration_samples = 0;       // 0 disables; otherwise average first N samples as zero offset
};

void declare_config(ManagerConfig & config);

// Shared logic: pressure (Pa) → optional startup zero-offset calibration →
// signed world-z depth → DepthFactor. Each specialized manager owns one of
// these; the templated SensorManagerBase scaffolding stays one-message-type-
// per-class (mirrors the dvl::Manager design).
class DepthProcessor
{
public:
  struct ProcessResult
  {
    gtsam::NonlinearFactorGraph factors;
    double measured_z;
  };

  DepthProcessor(const ManagerConfig & config, spdlog::logger * logger);

  // Returns the new factor graph + measured_z to declare, or std::nullopt
  // while the startup calibration is still gathering samples.
  std::optional<ProcessResult> process(double pressure_pa);

  // Latest valid measurement (world-z, with up positive). Updated on every
  // successful processed sample, including before the graph is initialized,
  // so that another sensor's init call can pull it through the registered
  // provider and anchor the first pose's z to the true depth.
  std::optional<double> getInitZHint();

private:
  const ManagerConfig & config_;
  spdlog::logger * logger_;

  std::vector<double> calibration_samples_;
  std::optional<double> pressure_offset_pa_;

  std::mutex z_mutex_;
  std::optional<double> latest_measured_z_;
};

// FluidPressure-source manager (sensor_msgs/FluidPressure).
class FluidPressureManager
: public SensorManagerBase<ManagerConfig, ri::SensorMsgsFluidPressure>
{
public:
  FluidPressureManager(
    const std::string & config_path, ri::NodeHandle & nh,
    mimosa::imu::Manager::SharedPtr imu_manager, mimosa::graph::Manager::SharedPtr graph_manager);
  void callback(const ri::ConstSharedPtr<ri::SensorMsgsFluidPressure> & msg) override;
  std::optional<double> getInitZHint() { return processor_.getInitZHint(); }

private:
  DepthProcessor processor_;
};

// Nortek-source manager (interfaces/BottomTrack.pressure on the same topic
// the DVL velocity manager already subscribes to).
class NortekBottomTrackManager
: public SensorManagerBase<ManagerConfig, ri::NortekBottomTrack>
{
public:
  NortekBottomTrackManager(
    const std::string & config_path, ri::NodeHandle & nh,
    mimosa::imu::Manager::SharedPtr imu_manager, mimosa::graph::Manager::SharedPtr graph_manager);
  void callback(const ri::ConstSharedPtr<ri::NortekBottomTrack> & msg) override;
  std::optional<double> getInitZHint() { return processor_.getInitZHint(); }

private:
  DepthProcessor processor_;
};

// Unified manager that creates the appropriate sub-manager based on config.
class Manager
{
public:
  Manager(
    const std::string & config_path, ri::NodeHandle & nh,
    mimosa::imu::Manager::SharedPtr imu_manager, mimosa::graph::Manager::SharedPtr graph_manager);

  std::string getSubscribedTopic() const
  {
    if (fluid_pressure_manager_) return fluid_pressure_manager_->getSubscribedTopic();
    if (nortek_manager_) return nortek_manager_->getSubscribedTopic();
    return "";
  }

  DepthSource getType() const { return type_; }

  void callbackFluidPressure(const ri::ConstSharedPtr<ri::SensorMsgsFluidPressure> & msg)
  {
    if (fluid_pressure_manager_) fluid_pressure_manager_->callback(msg);
  }

  void callbackNortekBottomTrack(const ri::ConstSharedPtr<ri::NortekBottomTrack> & msg)
  {
    if (nortek_manager_) nortek_manager_->callback(msg);
  }

private:
  DepthSource type_;
  std::unique_ptr<FluidPressureManager> fluid_pressure_manager_;
  std::unique_ptr<NortekBottomTrackManager> nortek_manager_;
};

}  // namespace depth
}  // namespace mimosa
