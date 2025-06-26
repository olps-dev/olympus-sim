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
   * Convert a point cloud to a Gazebo PointCloudPacked message
   * @param pointCloud The point cloud to convert
   * @param targetPose The target pose to transform points to
   * @param outMsg The output message
   */
  void ConvertToPointCloudMsg(
      const MmWavePointCloud &pointCloud,
      const gz::math::Pose3d &targetPose,
      gz::msgs::PointCloudPacked &outMsg);
      
  /**
   * Set up the header and field definitions for a point cloud message
   * @param pointCloud The point cloud providing metadata
   * @param outMsg The output message to configure
   */
  void SetupPointCloudMessageHeader(
      const MmWavePointCloud &pointCloud,
      gz::msgs::PointCloudPacked &outMsg);
      
  /**
   * Populate the point cloud message with transformed point data
   * @param pointCloud The point cloud providing the data
   * @param targetPose The target pose to transform points to
   * @param outMsg The output message to populate
   */
  void SetupPointCloudData(
      const MmWavePointCloud &pointCloud,
      const gz::math::Pose3d &targetPose,
      gz::msgs::PointCloudPacked &outMsg);
      
  /**
   * Add a single transformed point to the point cloud message data buffer
   * @param point The point to transform and add
   * @param transform The transformation to apply to the point
   * @param data The data buffer to add the point to
   * @param offset The byte offset in the data buffer
   */
  void AddTransformedPointToBuffer(
      const MmWavePoint &point,
      const gz::math::Pose3d &transform,
      std::string &data,
      unsigned int offset);
      
  /**
   * Load all point clouds from a directory
   * @param directory The directory containing point cloud files
   * @return True if at least one point cloud was loaded successfully, false otherwise
   */
  bool LoadPointCloudsFromDirectory(const std::string &directory);

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
   * Read and validate the file header of a point cloud file
   * @param file Input file stream positioned at the start of the file
   * @param filename Filename for error reporting
   * @return True if header is valid, false otherwise
   */
  bool ReadFileHeader(std::ifstream &file, const std::string &filename);
  
  /**
   * Read point cloud metadata (description, sensor parameters, etc.)
   * @param file Input file stream positioned after the header
   * @param pointCloud The point cloud object to populate
   * @return True if metadata was read successfully, false otherwise
   */
  bool ReadPointCloudMetadata(std::ifstream &file, std::shared_ptr<MmWavePointCloud> pointCloud);
  
  /**
   * Read point cloud data (all points)
   * @param file Input file stream positioned after metadata
   * @param pointCloud The point cloud object to populate
   * @return True if data was read successfully, false otherwise
   */
  bool ReadPointCloudData(std::ifstream &file, std::shared_ptr<MmWavePointCloud> pointCloud);
  
  /**
   * Read a single point from the file
   * @param file Input file stream positioned at a point
   * @param pointCloud The point cloud object to add the point to
   * @return True if point was read successfully, false otherwise
   */
  bool ReadSinglePoint(std::ifstream &file, std::shared_ptr<MmWavePointCloud> pointCloud);
  
  /**
   * Generate a default point cloud for basic scenarios
   * @param scenarioName The scenario to generate a point cloud for
   * @return A pointer to the generated point cloud, or nullptr on failure
   */
  std::shared_ptr<MmWavePointCloud> GenerateDefaultPointCloud(const std::string &scenarioName);
};

} // namespace olympus_sim

#endif // OLYMPUS_SIM_MMWAVE_POINT_CLOUD_LOADER_HH
