// Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include "mimosa/depth/factor.hpp"
#include "mimosa/sensor_manager_base.hpp"

namespace mimosa
{
namespace depth
{
struct ManagerConfig
{
  SensorManagerBaseConfig base;

  // Depth-specific fields
  float sigma_depth_m = 0.05;          // noise sigma for depth measurement [m]
  float fluid_density = 1025.0;        // [kg/m^3] (1000 freshwater, 1025 seawater)
  float surface_pressure = 101325.0;   // atmospheric pressure at the surface [Pa]
  float gravity_magnitude = 9.80665;   // [m/s^2]
};

void declare_config(ManagerConfig & config);

class Manager : public SensorManagerBase<ManagerConfig, ri::SensorMsgsFluidPressure>
{
public:
  Manager(
    const std::string & config_path, ri::NodeHandle & nh,
    mimosa::imu::Manager::SharedPtr imu_manager, mimosa::graph::Manager::SharedPtr graph_manager);
  void callback(const ri::ConstSharedPtr<ri::SensorMsgsFluidPressure> & msg) override;

private:
  // Offset captured from the first valid pressure message so that the first
  // depth factor has zero residual against the init pose (which mimosa places
  // at world-z = 0). All subsequent factors measure delta-z from that point.
  bool depth_offset_initialized_ = false;
  double depth_offset_ = 0.0;
};

}  // namespace depth
}  // namespace mimosa
