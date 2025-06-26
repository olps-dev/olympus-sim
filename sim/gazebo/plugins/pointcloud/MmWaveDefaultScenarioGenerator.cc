#include "MmWavePointCloudScenarioGenerators.hh"

#include <cmath>
#include <gz/common/Console.hh>
#include <gz/math/Rand.hh>

namespace olympus_sim
{

std::shared_ptr<MmWavePointCloud> DefaultScenarioGenerator::Generate(
    const std::string &scenarioName)
{
  auto pointCloud = std::make_shared<MmWavePointCloud>();
  pointCloud->name = scenarioName;
  pointCloud->description = "Default scene with mixed objects";
  
  InitializeDefaultSensorParams(pointCloud);
  
  // Generate generic scene with random objects
  int numPoints = 50;
  
  // Add mixed points
  for (int i = 0; i < numPoints; ++i) {
    MmWavePoint point;
    double angle = gz::math::Rand::DblUniform(0, 2 * M_PI);
    double distance = gz::math::Rand::DblUniform(1.0, 20.0);
    double height = gz::math::Rand::DblUniform(-1.0, 5.0);
    double x = distance * cos(angle);
    double y = distance * sin(angle);
    double z = height;
    
    point.position = gz::math::Vector3d(x, y, z);
    point.radialVelocity = gz::math::Rand::DblUniform(-5.0, 5.0);
    point.rcs = gz::math::Rand::DblUniform(0.1, 50.0);
    point.intensity = gz::math::Rand::DblUniform(0.1, 0.9);
    point.objectId = i / 5;
    point.classId = i % 5;
    
    pointCloud->points.push_back(point);
  }
  
  // Add noise to points
  AddNoiseToPoints(pointCloud, 0.05);  // 5cm noise

  gzmsg << "Generated default point cloud with " 
        << pointCloud->points.size() << " points." << std::endl;
        
  return pointCloud;
}

} // namespace olympus_sim
