#ifndef OLYMPUS_SIM_MMWAVE_SENSOR_WSL_COMPAT_HH_
#define OLYMPUS_SIM_MMWAVE_SENSOR_WSL_COMPAT_HH_

// Standard Gazebo Sim includes
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Types.hh> // Includes Entity
#include <gz/sim/Util.hh> // For worldPose
#include <gz/sim/components.hh> // Include all components

// Gazebo Math includes
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

// Gazebo Messages includes
#include <gz/msgs/pointcloud_packed.pb.h>

// Gazebo Common includes
#include <gz/common/Console.hh>

// Gazebo Math random number generation
#include <gz/math/Rand.hh>
#include <map>
#include <string>
#include <vector>

namespace olympus_sim
{
namespace wsl_compat
{

// Helper to calculate Radar Cross Section for an entity in WSL compat mode
double CalculateRCS(
    const gz::sim::Entity &_entity,
    const gz::sim::EntityComponentManager &_ecm,
    double defaultRCSValue,
    double minRCSValue);

// Helper to calculate radial velocity for an entity in WSL compat mode
double CalculateRadialVelocity(
    const gz::sim::Entity &_targetEntity,
    const gz::sim::Entity &_sensorEntity,
    const gz::math::Pose3d &_sensorPose,
    const gz::math::Vector3d &_rayDirection,
    const gz::sim::EntityComponentManager &_ecm,
    double maxRadialVelocityValue);

// Updates the simplified world model for WSL compatibility mode
void UpdateSimplifiedWorldModel(
    const gz::sim::Entity &_sensorEntity,
    const gz::sim::EntityComponentManager &_ecm,
    std::map<gz::sim::Entity, std::pair<gz::math::Pose3d, gz::math::Vector3d>> &simplifiedWorldModel,
    bool &wslWarningShown); // To update plugin's state

// Generates simulated sensor data using the simplified world model
void GenerateSimulatedData(
    const gz::sim::Entity &_sensorEntity,
    const gz::math::Pose3d &_sensorPose,
    const gz::sim::EntityComponentManager &_ecm,
    gz::msgs::PointCloudPacked &_pointCloudMsg,
    const std::map<gz::sim::Entity, std::pair<gz::math::Pose3d, gz::math::Vector3d>> &simplifiedWorldModel,
    double horizontalFov,
    int horizontalResolution,
    double verticalFov,
    int verticalResolution,
    double minRange,
    double maxRange,
    double noiseMean,
    double noiseStdDev,
    double defaultRCSValue,
    double minRCSValue,
    double maxRadialVelocity);

} // namespace wsl_compat
} // namespace olympus_sim

#endif // OLYMPUS_SIM_MMWAVE_SENSOR_WSL_COMPAT_HH_
