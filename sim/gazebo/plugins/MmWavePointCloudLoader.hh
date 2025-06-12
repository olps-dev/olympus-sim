#ifndef OLYMPUS_SIM_MMWAVE_POINT_CLOUD_LOADER_HH
#define OLYMPUS_SIM_MMWAVE_POINT_CLOUD_LOADER_HH

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <cstdint>

#include <gz/math/Vector3.hh>
#include <gz/math/Pose3.hh>
#include <gz/msgs/pointcloud_packed.pb.h>

namespace olympus_sim
{

/**
 * A struct to represent a single point in a mmWave point cloud
 */
struct MmWavePoint
{
  // Position of the point
  gz::math::Vector3d position;
  
  // Radial velocity (Doppler)
  float radialVelocity;
  
  // Radar Cross-Section
  float rcs;
  
  // Intensity/reflectivity
  float intensity;

  // Additional metadata
  uint32_t objectId;
  uint32_t classId;
};

/**
 * A struct to represent a pre-computed point cloud for a specific scenario
 */
struct MmWavePointCloud
{
  // The name/identifier of this point cloud
  std::string name;
  
  // Description/metadata
  std::string description;
  
  // The points in the point cloud
  std::vector<MmWavePoint> points;
  
  // The expected sensor pose this point cloud was created for
  gz::math::Pose3d sensorPose;

  // Sensor parameters used to generate this point cloud
  double horizontalFov;
  double verticalFov;
  int horizontalResolution;
  int verticalResolution;
};

/**
 * A class to load and manage pre-computed point clouds for the MmWave sensor plugin
 */
class MmWavePointCloudLoader
{
public:
  /**
   * Constructor
   */
  MmWavePointCloudLoader();

  /**
   * Initialize the point cloud loader
   * @param pointCloudDir The directory containing point cloud data
   */
  bool Initialize(const std::string &pointCloudDir);

  /**
   * Get a point cloud for a specific scenario
   * @param scenarioName The name of the scenario (e.g., "empty", "urban", "highway")
   * @param sensorPose The pose of the sensor
   * @return A pointer to the point cloud data, or nullptr if not found
   */
  std::shared_ptr<MmWavePointCloud> GetPointCloud(
      const std::string &scenarioName, 
      const gz::math::Pose3d &sensorPose);

  /**
   * Get the closest matching point cloud based on scene contents
   * @param sensorPose The pose of the sensor
   * @param objectTypes Map of object types and counts in the scene
   * @return A pointer to the best matching point cloud, or nullptr if none found
   */
  std::shared_ptr<MmWavePointCloud> GetClosestMatchingPointCloud(
      const gz::math::Pose3d &sensorPose,
      const std::map<std::string, int> &objectTypes);

  /**
   * Convert a point cloud to a PointCloudPacked message
   * @param pointCloud The point cloud to convert
   * @param targetPose The target pose for the point cloud (will transform points)
   * @param outMsg The output message to populate
   */
  void ConvertToPointCloudMsg(
      const MmWavePointCloud &pointCloud,
      const gz::math::Pose3d &targetPose,
      gz::msgs::PointCloudPacked &outMsg);

private:
  // Directory containing point cloud data
  std::string pointCloudDirectory;
  
  // Loaded point clouds
  std::map<std::string, std::shared_ptr<MmWavePointCloud>> pointClouds;
  
  /**
   * Load a point cloud from a file
   * @param filename The file to load
   * @return True if loaded successfully, false otherwise
   */
  bool LoadPointCloudFromFile(const std::string &filename);
  
  /**
   * Generate a default point cloud for basic scenarios
   * @param scenarioName The scenario to generate a point cloud for
   * @return A pointer to the generated point cloud, or nullptr on failure
   */
  std::shared_ptr<MmWavePointCloud> GenerateDefaultPointCloud(const std::string &scenarioName);
};

} // namespace olympus_sim

#endif // OLYMPUS_SIM_MMWAVE_POINT_CLOUD_LOADER_HH
