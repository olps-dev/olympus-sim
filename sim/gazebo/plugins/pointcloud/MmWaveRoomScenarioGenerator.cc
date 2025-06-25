#include "MmWavePointCloudScenarioGenerators.hh"

#include <cmath>
#include <vector>
#include <gz/common/Console.hh>
#include <gz/math/Rand.hh>

namespace olympus_sim
{

std::shared_ptr<MmWavePointCloud> RoomScenarioGenerator::Generate(
    const std::string &scenarioName)
{
  auto pointCloud = std::make_shared<MmWavePointCloud>();
  pointCloud->name = scenarioName;
  pointCloud->description = "Room scene with furniture and people";
  
  InitializeDefaultSensorParams(pointCloud);
  
  // Generate room contents
  GenerateFurniturePoints(pointCloud);
  GeneratePeoplePoints(pointCloud);
  
  // Add noise to points
  AddNoiseToPoints(pointCloud, 0.05);  // 5cm noise

  gzmsg << "Generated room scene point cloud with " 
        << pointCloud->points.size() << " points." << std::endl;
        
  return pointCloud;
}

void RoomScenarioGenerator::GenerateFurniturePoints(std::shared_ptr<MmWavePointCloud> pointCloud)
{
  // Room dimensions
  const double roomWidth = 5.0;   // meters
  const double roomLength = 6.0;   // meters
  const double roomHeight = 2.4;   // meters
  
  // Add furniture points
  const int numFurniture = 5;      // Typical room furniture count
  const int pointsPerFurniture = 8; // Points per furniture item
  
  // Furniture types and their dimensions (width, length, height)
  struct FurnitureType {
    std::string name;
    double width;
    double length;
    double height;
    double rcs;
  };
  
  std::vector<FurnitureType> furnitureTypes = {
    {"table",    1.2, 0.8, 0.75, 2.0},  // Dining/coffee table
    {"chair",    0.5, 0.5, 0.9,  1.5},  // Chair
    {"sofa",     2.0, 0.9, 0.8,  3.0},  // Sofa
    {"cabinet",  1.0, 0.4, 1.8,  2.5},  // Cabinet/shelf
    {"bed",      2.0, 1.6, 0.5,  2.8}   // Bed
  };
  
  // Place furniture items
  for (int f = 0; f < numFurniture; ++f) {
    // Select random furniture type
    const auto& furniture = furnitureTypes[f % furnitureTypes.size()];
    
    // Place furniture against walls or in corners with some variation
    double baseX, baseY;
    if (f % 2 == 0) { // Against walls
      baseX = gz::math::Rand::DblUniform(-roomWidth/2 + furniture.width/2,
                                        roomWidth/2 - furniture.width/2);
      baseY = (f % 4 < 2) ? -roomLength/2 + furniture.length/2 :
                             roomLength/2 - furniture.length/2;
    } else { // Interior placement
      baseX = gz::math::Rand::DblUniform(-roomWidth/3, roomWidth/3);
      baseY = gz::math::Rand::DblUniform(-roomLength/3, roomLength/3);
    }
  
    // Generate points for furniture surfaces
    for (int i = 0; i < pointsPerFurniture; ++i) {
      MmWavePoint point;
      
      // Different points for different furniture surfaces
      double localX, localY, localZ;
      switch (i % 4) {
        case 0: // Top surface
          localX = gz::math::Rand::DblUniform(-furniture.width/2, furniture.width/2);
          localY = gz::math::Rand::DblUniform(-furniture.length/2, furniture.length/2);
          localZ = furniture.height;
          break;
        case 1: // Front surface
          localX = gz::math::Rand::DblUniform(-furniture.width/2, furniture.width/2);
          localY = -furniture.length/2;
          localZ = gz::math::Rand::DblUniform(0, furniture.height);
          break;
        case 2: // Side surface
          localX = furniture.width/2;
          localY = gz::math::Rand::DblUniform(-furniture.length/2, furniture.length/2);
          localZ = gz::math::Rand::DblUniform(0, furniture.height);
          break;
        default: // Other side or back
          localX = -furniture.width/2;
          localY = gz::math::Rand::DblUniform(-furniture.length/2, furniture.length/2);
          localZ = gz::math::Rand::DblUniform(0, furniture.height);
          break;
      }
      
      // Transform to world coordinates
      point.position = gz::math::Vector3d(
        baseX + localX,
        baseY + localY,
        localZ
      );
      
      point.radialVelocity = 0.0f;
      point.rcs = furniture.rcs * gz::math::Rand::DblUniform(0.8, 1.2);
      point.intensity = gz::math::Rand::DblUniform(0.4, 0.7);
      point.objectId = 100 + f;  // Unique ID per furniture
      point.classId = 2;         // Furniture class
      
      pointCloud->points.push_back(point);
    }
  }
}

void RoomScenarioGenerator::GeneratePeoplePoints(std::shared_ptr<MmWavePointCloud> pointCloud)
{
  // Room dimensions
  const double roomWidth = 5.0;   // meters
  const double roomLength = 6.0;   // meters
  
  // Add people points
  const int numPeople = 2;  // Typical number of people in a room
  const int pointsPerPerson = 6;
  
  for (int p = 0; p < numPeople; ++p) {
    // Random position in room, avoiding furniture
    double baseX = gz::math::Rand::DblUniform(-roomWidth/3, roomWidth/3);
    double baseY = gz::math::Rand::DblUniform(-roomLength/3, roomLength/3);
    
    // Generate points for different body parts
    for (int i = 0; i < pointsPerPerson; ++i) {
      MmWavePoint point;
      
      // Different points for different body parts
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
        case 2: // Lower torso
          localX = gz::math::Rand::DblUniform(-0.2, 0.2);
          localY = gz::math::Rand::DblUniform(-0.1, 0.1);
          localZ = 1.0 + gz::math::Rand::DblUniform(-0.1, 0.1);
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
      
      // People might be moving slowly
      double moveSpeed = gz::math::Rand::DblUniform(0.0, 0.5);
      gz::math::Vector3d moveDir(gz::math::Rand::DblUniform(-1.0, 1.0),
                                gz::math::Rand::DblUniform(-1.0, 1.0),
                                0.0);
      moveDir.Normalize();
      point.radialVelocity = moveSpeed * moveDir.Dot(point.position.Normalized());
      
      // RCS varies by body part
      point.rcs = (i <= 2) ? 
        gz::math::Rand::DblUniform(0.8, 1.5) :  // Torso/head
        gz::math::Rand::DblUniform(0.3, 0.8);   // Limbs
      
      point.intensity = gz::math::Rand::DblUniform(0.3, 0.6);
      point.objectId = 200 + p;  // Unique ID per person
      point.classId = 3;         // Person class
      
      pointCloud->points.push_back(point);
    }
  }
}

void RoomScenarioGenerator::GenerateVehiclePoints(std::shared_ptr<MmWavePointCloud> pointCloud)
{
  const int numVehicles = 8;
  const int pointsPerVehicle = 12;
  
  // Create vehicles on road-like paths
  for (int v = 0; v < numVehicles; ++v) {
    // Determine if vehicle is on main road
    bool onMainRoad = (v < numVehicles/2);
    
    // Base position and orientation
    double baseAngle;
    double baseDistance;
    if (onMainRoad) {
      // Vehicles on main road follow a straight path
      baseAngle = (v % 2) ? 0 : M_PI; // Alternating directions
      baseDistance = gz::math::Rand::DblUniform(5.0, 20.0);
    } else {
      // Other vehicles on side roads or parking
      baseAngle = gz::math::Rand::DblUniform(0, 2 * M_PI);
      baseDistance = gz::math::Rand::DblUniform(8.0, 15.0);
    }
    
    double baseX = baseDistance * cos(baseAngle);
    double baseY = baseDistance * sin(baseAngle);
    
    // Vehicle dimensions (randomly sized)
    double vehLength = gz::math::Rand::DblUniform(4.0, 5.0);
    double vehWidth = gz::math::Rand::DblUniform(1.8, 2.0);
    double vehHeight = gz::math::Rand::DblUniform(1.4, 1.8);
    
    // Generate points for vehicle surfaces
    for (int i = 0; i < pointsPerVehicle; ++i) {
      MmWavePoint point;
      
      // Different points for different vehicle parts
      double localX, localY, localZ;
      switch (i % 6) {
        case 0: // Front
          localX = vehLength/2;
          localY = gz::math::Rand::DblUniform(-vehWidth/2, vehWidth/2);
          localZ = gz::math::Rand::DblUniform(0.2, vehHeight);
          break;
        case 1: // Back
          localX = -vehLength/2;
          localY = gz::math::Rand::DblUniform(-vehWidth/2, vehWidth/2);
          localZ = gz::math::Rand::DblUniform(0.2, vehHeight);
          break;
        case 2: // Right side
          localX = gz::math::Rand::DblUniform(-vehLength/2, vehLength/2);
          localY = vehWidth/2;
          localZ = gz::math::Rand::DblUniform(0.2, vehHeight);
          break;
        case 3: // Left side
          localX = gz::math::Rand::DblUniform(-vehLength/2, vehLength/2);
          localY = -vehWidth/2;
          localZ = gz::math::Rand::DblUniform(0.2, vehHeight);
          break;
        case 4: // Top
          localX = gz::math::Rand::DblUniform(-vehLength/2, vehLength/2);
          localY = gz::math::Rand::DblUniform(-vehWidth/2, vehWidth/2);
          localZ = vehHeight;
          break;
        default: // Bottom/wheels
          localX = ((i % 2) ? 1 : -1) * vehLength/3;
          localY = ((i % 4 < 2) ? 1 : -1) * vehWidth/2;
          localZ = 0.3; // Wheel height
      }
      
      // Transform to world coordinates with vehicle orientation
      double cosAngle = cos(baseAngle);
      double sinAngle = sin(baseAngle);
      point.position = gz::math::Vector3d(
        baseX + (localX * cosAngle - localY * sinAngle),
        baseY + (localX * sinAngle + localY * cosAngle),
        localZ
      );
      
      // Vehicle speed based on location
      double speed;
      if (onMainRoad) {
        speed = gz::math::Rand::DblUniform(8.0, 15.0);
        if (baseAngle > M_PI/2 && baseAngle < 3*M_PI/2) {
          speed *= -1; // Opposite direction
        }
      } else {
        speed = gz::math::Rand::DblUniform(-2.0, 5.0);
      }
      
      // Project velocity onto radial direction
      gz::math::Vector3d moveDir(cos(baseAngle), sin(baseAngle), 0);
      point.radialVelocity = speed * moveDir.Dot(
        point.position.Normalized());
      
      // RCS varies by vehicle part and material
      switch (i % 6) {
        case 0: // Front (grille, bumper)
          point.rcs = gz::math::Rand::DblUniform(15.0, 25.0);
          break;
        case 1: // Back (tail lights, bumper)
          point.rcs = gz::math::Rand::DblUniform(12.0, 20.0);
          break;
        case 2: // Right side
        case 3: // Left side
          point.rcs = gz::math::Rand::DblUniform(8.0, 15.0);
          break;
        case 4: // Top
          point.rcs = gz::math::Rand::DblUniform(5.0, 10.0);
          break;
        default: // Wheels (strong reflectors)
          point.rcs = gz::math::Rand::DblUniform(18.0, 30.0);
      }
      
      point.intensity = gz::math::Rand::DblUniform(0.6, 0.9);
      point.objectId = 200 + v;  // Unique ID per vehicle
      point.classId = 3;         // Vehicle class
      
      pointCloud->points.push_back(point);
    }
  }
}

} // namespace olympus_sim
