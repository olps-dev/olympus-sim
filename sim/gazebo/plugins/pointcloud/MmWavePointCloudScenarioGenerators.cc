#include "MmWavePointCloudScenarioGenerators.hh"

#include <cmath>
#include <gz/common/Console.hh>
#include <gz/math/Rand.hh>

namespace olympus_sim
{

// --------------- Factory Implementation ---------------

std::shared_ptr<MmWavePointCloudGenerator> MmWavePointCloudGeneratorFactory::CreateGenerator(
    const std::string &scenarioName)
{
  if (scenarioName == "empty") {
    return std::make_shared<EmptyScenarioGenerator>();
  } else if (scenarioName == "room") {
    return std::make_shared<RoomScenarioGenerator>();
  } else if (scenarioName == "bedroom") {
    return std::make_shared<BedroomScenarioGenerator>();
  } else {
    return std::make_shared<DefaultScenarioGenerator>();
  }
}

// --------------- Empty Scenario Generator ---------------

std::shared_ptr<MmWavePointCloud> EmptyScenarioGenerator::Generate(
    const std::string &scenarioName)
{
  auto pointCloud = std::make_shared<MmWavePointCloud>();
  pointCloud->name = scenarioName;
  pointCloud->description = "Empty room with floor and wall points";
  
  InitializeDefaultSensorParams(pointCloud);
  
  // Room dimensions
  const double roomWidth = 5.0;   // meters
  const double roomLength = 6.0;   // meters
  const double roomHeight = 2.4;   // meters
  
  // Add floor points
  for (int i = 0; i < 20; ++i) {
    MmWavePoint point;
    double x = gz::math::Rand::DblUniform(-roomWidth/2, roomWidth/2);
    double y = gz::math::Rand::DblUniform(-roomLength/2, roomLength/2);
    double z = 0.0;
    
    point.position = gz::math::Vector3d(x, y, z);
    point.radialVelocity = 0.0f;
    point.rcs = gz::math::Rand::DblUniform(0.5, 2.0);        // Floor material
    point.intensity = gz::math::Rand::DblUniform(0.2, 0.4);  // Floor reflectivity
    point.objectId = 0;  // Floor points
    point.classId = 0;   // Floor class
    
    pointCloud->points.push_back(point);
  }
  
  // Add wall points
  for (int i = 0; i < 20; ++i) {
    MmWavePoint point;
    
    // Decide which wall this point belongs to
    int wall = i % 4;
    double x, y, z;
    
    switch(wall) {
      case 0: // Front wall
        x = gz::math::Rand::DblUniform(-roomWidth/2, roomWidth/2);
        y = -roomLength/2;
        z = gz::math::Rand::DblUniform(0, roomHeight);
        break;
      case 1: // Right wall
        x = roomWidth/2;
        y = gz::math::Rand::DblUniform(-roomLength/2, roomLength/2);
        z = gz::math::Rand::DblUniform(0, roomHeight);
        break;
      case 2: // Back wall
        x = gz::math::Rand::DblUniform(-roomWidth/2, roomWidth/2);
        y = roomLength/2;
        z = gz::math::Rand::DblUniform(0, roomHeight);
        break;
      default: // Left wall
        x = -roomWidth/2;
        y = gz::math::Rand::DblUniform(-roomLength/2, roomLength/2);
        z = gz::math::Rand::DblUniform(0, roomHeight);
        break;
    }
    
    point.position = gz::math::Vector3d(x, y, z);
    point.radialVelocity = 0.0f;
    point.rcs = gz::math::Rand::DblUniform(1.0, 3.0);        // Wall material
    point.intensity = gz::math::Rand::DblUniform(0.3, 0.5);  // Wall reflectivity
    point.objectId = 1;  // Wall points
    point.classId = 1;   // Wall class
    
    pointCloud->points.push_back(point);
  }

  // Add noise to points
  AddNoiseToPoints(pointCloud, 0.05);  // 5cm noise

  gzmsg << "Generated empty room point cloud with " 
        << pointCloud->points.size() << " points." << std::endl;
        
  return pointCloud;
}

} // namespace olympus_sim
