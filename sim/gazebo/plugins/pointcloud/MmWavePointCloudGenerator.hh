#ifndef OLYMPUS_SIM_MMWAVE_POINT_CLOUD_GENERATOR_HH
#define OLYMPUS_SIM_MMWAVE_POINT_CLOUD_GENERATOR_HH

#include <memory>
#include <string>

#include "MmWavePointCloudLoader.hh"

namespace olympus_sim
{

/**
 * Abstract base class for point cloud generators.
 * Each derived class implements generation for a specific scenario.
 */
class MmWavePointCloudGenerator
{
public:
  /**
   * Constructor
   */
  MmWavePointCloudGenerator() = default;
  
  /**
   * Virtual destructor
   */
  virtual ~MmWavePointCloudGenerator() = default;
  
  /**
   * Generate a point cloud for a specific scenario
   * @param scenarioName The name of the scenario
   * @return A pointer to the generated point cloud
   */
  virtual std::shared_ptr<MmWavePointCloud> Generate(
      const std::string &scenarioName) = 0;

protected:
  /**
   * Initialize default sensor parameters
   * @param pointCloud The point cloud to initialize
   */
  void InitializeDefaultSensorParams(std::shared_ptr<MmWavePointCloud> pointCloud);
  
  /**
   * Add noise to points in the point cloud
   * @param pointCloud The point cloud to modify
   * @param rangeNoise The noise standard deviation
   */
  void AddNoiseToPoints(std::shared_ptr<MmWavePointCloud> pointCloud, double rangeNoise);
};

} // namespace olympus_sim

#endif // OLYMPUS_SIM_MMWAVE_POINT_CLOUD_GENERATOR_HH
