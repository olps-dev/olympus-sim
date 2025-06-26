#include "MmWavePointCloudScenarioGenerators.hh"

#include <cmath>
#include <gz/common/Console.hh>
#include <gz/math/Rand.hh>

namespace olympus_sim
{

std::shared_ptr<MmWavePointCloud> BedroomScenarioGenerator::Generate(
    const std::string &scenarioName)
{
  auto pointCloud = std::make_shared<MmWavePointCloud>();
  pointCloud->name = scenarioName;
  pointCloud->description = "Bedroom scene with bed, furniture, and possible occupant";
  
  InitializeDefaultSensorParams(pointCloud);
  
  // Generate bedroom contents
  GenerateBedPoints(pointCloud);
  GenerateBedroomFurniturePoints(pointCloud);
  GenerateOccupantPoints(pointCloud);
  
  // Add noise to points
  AddNoiseToPoints(pointCloud, 0.05);  // 5cm noise

  gzmsg << "Generated bedroom scene point cloud with " 
        << pointCloud->points.size() << " points." << std::endl;
        
  return pointCloud;
}

void BedroomScenarioGenerator::GenerateBedPoints(std::shared_ptr<MmWavePointCloud> pointCloud)
{
  // Room dimensions
  const double roomWidth = 4.0;   // meters
  const double roomLength = 5.0;   // meters
  
  // Bed parameters
  double bedWidth = 1.6;
  double bedLength = 2.0;
  double bedHeight = 0.5;
  double baseX = roomWidth/4;  // Position bed against wall
  double baseY = 0;
  
  // Generate points for bed
  for (int i = 0; i < 10; ++i) {
    MmWavePoint point;
    
    // Different points for bed surfaces
    double localX, localY, localZ;
    switch (i % 5) {
      case 0: // Top surface
        localX = gz::math::Rand::DblUniform(-bedWidth/2, bedWidth/2);
        localY = gz::math::Rand::DblUniform(-bedLength/2, bedLength/2);
        localZ = bedHeight;
        break;
      case 1: // Front
        localX = gz::math::Rand::DblUniform(-bedWidth/2, bedWidth/2);
        localY = -bedLength/2;
        localZ = gz::math::Rand::DblUniform(0, bedHeight);
        break;
      case 2: // Back
        localX = gz::math::Rand::DblUniform(-bedWidth/2, bedWidth/2);
        localY = bedLength/2;
        localZ = gz::math::Rand::DblUniform(0, bedHeight);
        break;
      case 3: // Left side
        localX = -bedWidth/2;
        localY = gz::math::Rand::DblUniform(-bedLength/2, bedLength/2);
        localZ = gz::math::Rand::DblUniform(0, bedHeight);
        break;
      default: // Right side
        localX = bedWidth/2;
        localY = gz::math::Rand::DblUniform(-bedLength/2, bedLength/2);
        localZ = gz::math::Rand::DblUniform(0, bedHeight);
    }
    
    point.position = gz::math::Vector3d(
      baseX + localX,
      baseY + localY,
      localZ
    );
    
    point.radialVelocity = 0.0f;
    point.rcs = gz::math::Rand::DblUniform(2.0, 4.0);  // Bed material
    point.intensity = gz::math::Rand::DblUniform(0.4, 0.7);
    point.objectId = 100;  // Bed ID
    point.classId = 2;     // Furniture class
    
    pointCloud->points.push_back(point);
  }
}

void BedroomScenarioGenerator::GenerateBedroomFurniturePoints(std::shared_ptr<MmWavePointCloud> pointCloud)
{
  // Room dimensions
  const double roomWidth = 4.0;   // meters
  const double roomLength = 5.0;   // meters
  
  // Add other bedroom furniture (dresser, nightstand)
  const int numFurniture = 2;
  for (int f = 0; f < numFurniture; ++f) {
    // Furniture dimensions
    double width = (f == 0) ? 1.2 : 0.5;  // Dresser or nightstand
    double length = (f == 0) ? 0.5 : 0.5;
    double height = (f == 0) ? 1.2 : 0.6;
    
    // Position against walls
    double baseX = (f == 0) ? -roomWidth/3 : roomWidth/4 + 1.0;
    double baseY = (f == 0) ? -roomLength/3 : 1.2;
    
    // Generate points for furniture
    for (int i = 0; i < 6; ++i) {
      MmWavePoint point;
      
      // Different points for furniture surfaces
      double localX, localY, localZ;
      switch (i % 3) {
        case 0: // Top
          localX = gz::math::Rand::DblUniform(-width/2, width/2);
          localY = gz::math::Rand::DblUniform(-length/2, length/2);
          localZ = height;
          break;
        case 1: // Front
          localX = gz::math::Rand::DblUniform(-width/2, width/2);
          localY = -length/2;
          localZ = gz::math::Rand::DblUniform(0, height);
          break;
        default: // Side
          localX = width/2;
          localY = gz::math::Rand::DblUniform(-length/2, length/2);
          localZ = gz::math::Rand::DblUniform(0, height);
      }
      
      point.position = gz::math::Vector3d(
        baseX + localX,
        baseY + localY,
        localZ
      );
      
      point.radialVelocity = 0.0f;
      point.rcs = gz::math::Rand::DblUniform(1.5, 3.0);  // Furniture material
      point.intensity = gz::math::Rand::DblUniform(0.4, 0.6);
      point.objectId = 101 + f;  // Unique furniture ID
      point.classId = 2;         // Furniture class
      
      pointCloud->points.push_back(point);
    }
  }
}

void BedroomScenarioGenerator::GenerateOccupantPoints(std::shared_ptr<MmWavePointCloud> pointCloud)
{
  // Room dimensions
  const double roomWidth = 4.0;   // meters
  
  // Randomly add a person (50% chance)
  if (gz::math::Rand::DblUniform(0, 1) > 0.5) {
    // Person position near bed
    double baseX = roomWidth/4;
    double baseY = 0.5;
    
    // Generate points for person
    for (int i = 0; i < 6; ++i) {
      MmWavePoint point;
      
      // Different points for body parts
      double localX, localY, localZ;
      switch (i) {
        case 0: // Head
          localX = gz::math::Rand::DblUniform(-0.1, 0.1);
          localY = gz::math::Rand::DblUniform(-0.1, 0.1);
          localZ = 1.6 + gz::math::Rand::DblUniform(-0.1, 0.1);
          break;
        case 1: // Upper torso
          localX = gz::math::Rand::DblUniform(-0.2, 0.2);
          localY = gz::math::Rand::DblUniform(-0.1, 0.1);
          localZ = 1.3 + gz::math::Rand::DblUniform(-0.1, 0.1);
          break;
        default: // Limbs
          localX = gz::math::Rand::DblUniform(-0.3, 0.3);
          localY = gz::math::Rand::DblUniform(-0.2, 0.2);
          localZ = 0.5 + gz::math::Rand::DblUniform(0.0, 1.0);
      }
      
      point.position = gz::math::Vector3d(
        baseX + localX,
        baseY + localY,
        localZ
      );
      
      // Minimal movement for person in bedroom
      point.radialVelocity = gz::math::Rand::DblUniform(-0.2, 0.2);
      
      // RCS varies by body part
      point.rcs = (i <= 1) ? 
        gz::math::Rand::DblUniform(0.8, 1.5) :  // Torso/head
        gz::math::Rand::DblUniform(0.3, 0.8);   // Limbs
      
      point.intensity = gz::math::Rand::DblUniform(0.3, 0.6);
      point.objectId = 200;  // Person ID
      point.classId = 3;     // Person class
      
      pointCloud->points.push_back(point);
    }
  }
}

} // namespace olympus_sim
