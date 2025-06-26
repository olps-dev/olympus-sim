#ifndef OLYMPUS_SIM_MMWAVE_SENSOR_RAY_HH_
#define OLYMPUS_SIM_MMWAVE_SENSOR_RAY_HH_

#include <gz/math/Pose3.hh>
#include <gz/math/Rand.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/rendering/RayQuery.hh>
#include <gz/rendering/Scene.hh>
#include <random>

namespace olympus_sim
{

/**
 * @brief Class to handle ray casting operations for MmWave sensor
 * 
 * This class abstracts the ray casting functionality
 */
class MmWaveSensorRay
{
public:
  /**
   * @brief Constructor
   */
  MmWaveSensorRay();

  /**
   * @brief Set the ray query to use for ray casting
   * @param rayQuery Ray query object to use
   */
  void SetRayQuery(const gz::rendering::RayQueryPtr &rayQuery);
  
  /**
   * @brief Perform ray casting to generate sensor data
   * @param sensorPose Pose of the sensor
   * @param ecm Entity component manager
   * @param pointCloudMsg Message to populate with point cloud data
   * @param horizontalFov Horizontal field of view in radians
   * @param horizontalResolution Horizontal resolution in degrees
   * @param verticalFov Vertical field of view in radians
   * @param verticalResolution Vertical resolution in degrees
   * @param minRange Minimum range in meters
   * @param maxRange Maximum range in meters
   * @param noiseMean Mean of noise distribution
   * @param noiseStdDev Standard deviation of noise distribution
   * @param defaultRCS Default radar cross-section
   * @param minRCS Minimum radar cross-section
   * @param maxRadialVelocity Maximum radial velocity
   * @param gaussianNoise Gaussian noise generator
   */
  void CastRays(
      const gz::math::Pose3d &sensorPose,
      const gz::sim::EntityComponentManager &ecm,
      gz::msgs::PointCloudPacked &pointCloudMsg,
      double horizontalFov,
      double horizontalResolution,
      double verticalFov,
      double verticalResolution,
      double minRange,
      double maxRange,
      double noiseMean,
      double noiseStdDev,
      double defaultRCS,
      double minRCS,
      double maxRadialVelocity,
      std::normal_distribution<double> &gaussianNoise);

private:
  /// Ray query interface for ray casting
  gz::rendering::RayQueryPtr rayQuery;
};

}  // namespace olympus_sim

#endif  // OLYMPUS_SIM_MMWAVE_SENSOR_RAY_HH_
