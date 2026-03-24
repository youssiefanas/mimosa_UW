// Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

// mimosa
#include "mimosa/DVL/factor.hpp"
#include "mimosa/sensor_manager_base.hpp"

namespace mimosa
{
namespace dvl
{

enum class DVLType
{
  Waterlinked = 0,
  Nortek = 1
};

struct ManagerConfig
{
  SensorManagerBaseConfig base;

  // DVL-specific fields
  int dvl_type = 0;         // 0 = Waterlinked, 1 = Nortek
  float fom_scale = 1.0;    // noise sigma = fom * fom_scale [m/s]
  float max_fom = 0.5;      // reject measurements with FOM above this [m/s]
  float max_velocity = 10.0; // reject unreasonable velocities [m/s]
};

void declare_config(ManagerConfig & config);

// Waterlinked DVL manager (has std_msgs/Header)
class WaterlinkedManager
: public SensorManagerBase<ManagerConfig, ri::WaterlinkedDVL>
{
public:
  WaterlinkedManager(
    const std::string & config_path, ri::NodeHandle & nh,
    mimosa::imu::Manager::SharedPtr imu_manager, mimosa::graph::Manager::SharedPtr graph_manager);
  void callback(const ri::ConstSharedPtr<ri::WaterlinkedDVL> & msg) override;

private:
  void processVelocity(
    const gtsam::Vector3 & vel_sensor, const gtsam::Vector3 & noise_sigmas, double timestamp);
};

// Nortek DVL manager (uses builtin_interfaces/Time system_timestamp, no std_msgs/Header)
class NortekManager
: public SensorManagerBase<ManagerConfig, ri::NortekBottomTrack>
{
public:
  NortekManager(
    const std::string & config_path, ri::NodeHandle & nh,
    mimosa::imu::Manager::SharedPtr imu_manager, mimosa::graph::Manager::SharedPtr graph_manager);
  void callback(const ri::ConstSharedPtr<ri::NortekBottomTrack> & msg) override;

private:
  void processVelocity(
    const gtsam::Vector3 & vel_sensor, const gtsam::Vector3 & noise_sigmas, double timestamp);
};

// Unified manager that creates the appropriate sub-manager based on config
class Manager
{
public:
  Manager(
    const std::string & config_path, ri::NodeHandle & nh,
    mimosa::imu::Manager::SharedPtr imu_manager, mimosa::graph::Manager::SharedPtr graph_manager);

  std::string getSubscribedTopic() const
  {
    if (waterlinked_manager_) return waterlinked_manager_->getSubscribedTopic();
    if (nortek_manager_) return nortek_manager_->getSubscribedTopic();
    return "";
  }

  DVLType getType() const { return type_; }

  void callbackWaterlinked(const ri::ConstSharedPtr<ri::WaterlinkedDVL> & msg)
  {
    if (waterlinked_manager_) waterlinked_manager_->callback(msg);
  }

  void callbackNortek(const ri::ConstSharedPtr<ri::NortekBottomTrack> & msg)
  {
    if (nortek_manager_) nortek_manager_->callback(msg);
  }

private:
  DVLType type_;
  std::unique_ptr<WaterlinkedManager> waterlinked_manager_;
  std::unique_ptr<NortekManager> nortek_manager_;
};

}  // namespace dvl
}  // namespace mimosa
