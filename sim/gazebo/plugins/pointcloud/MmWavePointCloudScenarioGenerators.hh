#ifndef OLYMPUS_SIM_MMWAVE_POINT_CLOUD_SCENARIO_GENERATORS_HH
#define OLYMPUS_SIM_MMWAVE_POINT_CLOUD_SCENARIO_GENERATORS_HH

#include "MmWavePointCloudGenerator.hh"
#include <memory>
#include <string>

namespace olympus_sim
{

/**
 * Point cloud generator for empty room scenarios
 */
class EmptyScenarioGenerator : public MmWavePointCloudGenerator
{
public:
  std::shared_ptr<MmWavePointCloud> Generate(
      const std::string &scenarioName) override;
};

/**
 * Point cloud generator for room scenarios with furniture
 */
class RoomScenarioGenerator : public MmWavePointCloudGenerator
{
public:
  std::shared_ptr<MmWavePointCloud> Generate(
      const std::string &scenarioName) override;
      
private:
  /**
   * Generate furniture points for a room
   * @param pointCloud The point cloud to add points to
   */
  void GenerateFurniturePoints(std::shared_ptr<MmWavePointCloud> pointCloud);
  
  /**
   * Generate people points for a room
   * @param pointCloud The point cloud to add points to
   */
  void GeneratePeoplePoints(std::shared_ptr<MmWavePointCloud> pointCloud);
  
  /**
   * Generate vehicle points for outdoor scenes
   * @param pointCloud The point cloud to add points to
   */
  void GenerateVehiclePoints(std::shared_ptr<MmWavePointCloud> pointCloud);
};

/**
 * Point cloud generator for bedroom scenarios
 */
class BedroomScenarioGenerator : public MmWavePointCloudGenerator
{
public:
  std::shared_ptr<MmWavePointCloud> Generate(
      const std::string &scenarioName) override;
      
private:
  /**
   * Generate bed points
   * @param pointCloud The point cloud to add points to
   */
  void GenerateBedPoints(std::shared_ptr<MmWavePointCloud> pointCloud);
  
  /**
   * Generate furniture points specific to bedroom
   * @param pointCloud The point cloud to add points to
   */
  void GenerateBedroomFurniturePoints(std::shared_ptr<MmWavePointCloud> pointCloud);
  
  /**
   * Generate occupant points
   * @param pointCloud The point cloud to add points to
   */
  void GenerateOccupantPoints(std::shared_ptr<MmWavePointCloud> pointCloud);
};

/**
 * Point cloud generator for default/fallback scenarios
 */
class DefaultScenarioGenerator : public MmWavePointCloudGenerator
{
public:
  std::shared_ptr<MmWavePointCloud> Generate(
      const std::string &scenarioName) override;
};

/**
 * Factory for creating scenario generators
 */
class MmWavePointCloudGeneratorFactory
{
public:
  /**
   * Create a generator for the specified scenario type
   * @param scenarioName The name of the scenario
   * @return A pointer to the appropriate generator
   */
  static std::shared_ptr<MmWavePointCloudGenerator> CreateGenerator(
      const std::string &scenarioName);
};

} // namespace olympus_sim

#endif // OLYMPUS_SIM_MMWAVE_POINT_CLOUD_SCENARIO_GENERATORS_HH
