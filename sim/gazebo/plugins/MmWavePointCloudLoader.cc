#include "MmWavePointCloudLoader.hh"

#include <fstream>
#include <sstream>
#include <filesystem>
#include <algorithm>
#include <cstring>
#include <cmath>

#include <gz/common/Console.hh>
#include <gz/math/Rand.hh>

namespace olympus_sim
{

MmWavePointCloudLoader::MmWavePointCloudLoader()
  : pointCloudDirectory("")
{
}

bool MmWavePointCloudLoader::Initialize(const std::string &pointCloudDir)
{
  pointCloudDirectory = pointCloudDir;
  
  // Check if directory exists
  if (!std::filesystem::exists(pointCloudDirectory)) {
    gzerr << "Point cloud directory does not exist: " << pointCloudDirectory << std::endl;
    return false;
  }
  
  gzmsg << "Initializing MmWave point cloud loader with directory: " << pointCloudDirectory << std::endl;
  
  // Try to load point clouds from files
  bool foundAny = false;
  try {
    for (const auto & entry : std::filesystem::directory_iterator(pointCloudDirectory)) {
      if (entry.path().extension() == ".mmpc") {
        gzmsg << "Found point cloud file: " << entry.path().string() << std::endl;
        if (LoadPointCloudFromFile(entry.path().string())) {
          foundAny = true;
        }
      }
    }
  } catch (const std::exception& e) {
    gzerr << "Error loading point cloud files: " << e.what() << std::endl;
  }

  // If no point clouds found, generate default ones
  if (!foundAny) {
    gzmsg << "No point cloud files found. Generating default point clouds." << std::endl;
    
    // Generate default point clouds for common scenarios
    auto emptyCloud = GenerateDefaultPointCloud("empty");
    if (emptyCloud) {
      pointClouds["empty"] = emptyCloud;
      foundAny = true;
    }
    
    auto roomCloud = GenerateDefaultPointCloud("room");
    if (roomCloud) {
      pointClouds["room"] = roomCloud;
      foundAny = true;
    }

    auto bedroomCloud = GenerateDefaultPointCloud("bedroom");
    if (bedroomCloud) {
      pointClouds["bedroom"] = bedroomCloud;
      foundAny = true;
    }
  }
  
  gzmsg << "MmWave point cloud loader initialized with " 
        << pointClouds.size() << " point clouds." << std::endl;
  
  return foundAny;
}

std::shared_ptr<MmWavePointCloud> MmWavePointCloudLoader::GetPointCloud(
    const std::string &scenarioName, 
    const gz::math::Pose3d &sensorPose)
{
  auto it = pointClouds.find(scenarioName);
  if (it != pointClouds.end()) {
    return it->second;
  }
  
  // If not found, try to generate a default one
  gzmsg << "Point cloud for scenario '" << scenarioName 
        << "' not found. Generating default." << std::endl;
  
  return GenerateDefaultPointCloud(scenarioName);
}

std::shared_ptr<MmWavePointCloud> MmWavePointCloudLoader::GetClosestMatchingPointCloud(
    const gz::math::Pose3d &sensorPose,
    const std::map<std::string, int> &objectTypes)
{
  // If we have no point clouds, generate a default one
  if (pointClouds.empty()) {
    gzmsg << "No point clouds available. Generating default." << std::endl;
    return GenerateDefaultPointCloud("default");
  }

  // New heuristic for indoor environments:
  // 1. If no objects, use "empty".
  // 2. If people are present, use "room" or "bedroom".
  // 3. If furniture is present, use "room".
  int personCount = 0;
  int furnitureCount = 0;

  for (const auto& [type, count] : objectTypes) {
    if (type.find("person") != std::string::npos || 
        type.find("human") != std::string::npos) {
      personCount += count;
    } else if (type.find("table") != std::string::npos || 
               type.find("chair") != std::string::npos ||
               type.find("sofa") != std::string::npos ||
               type.find("bed") != std::string::npos) {
      furnitureCount += count;
    }
  }

  // If people are detected, a "bedroom" or "room" is most likely.
  // Prefer "bedroom" if available and people are present.
  if (personCount > 0) {
    auto it = pointClouds.find("bedroom");
    if (it != pointClouds.end()) {
      return it->second;
    }
    it = pointClouds.find("room");
    if (it != pointClouds.end()) {
        return it->second;
    }
  }

  // If furniture is present, a "room" is likely.
  if (furnitureCount > 0) {
    auto it = pointClouds.find("room");
    if (it != pointClouds.end()) {
      return it->second;
    }
  }

  // If no specific objects are detected, use "empty".
  if (personCount == 0 && furnitureCount == 0) {
    auto it = pointClouds.find("empty");
    if (it != pointClouds.end()) {
      return it->second;
    }
  }

  // If no specific match, just use the first available
  if (!pointClouds.empty()) {
    return pointClouds.begin()->second;
  }

  // If still nothing, generate a default
  return GenerateDefaultPointCloud("default");
}

void MmWavePointCloudLoader::ConvertToPointCloudMsg(
    const MmWavePointCloud &pointCloud,
    const gz::math::Pose3d &targetPose,
    gz::msgs::PointCloudPacked &outMsg)
{
  // Calculate the transform from the point cloud's sensor pose to the target pose
  gz::math::Pose3d transform = targetPose.Inverse() * pointCloud.sensorPose;
  
  // Set point cloud message properties
  outMsg.set_width(pointCloud.points.size());
  outMsg.set_height(1);
  outMsg.set_is_dense(true); // Assuming all points are valid
  outMsg.set_point_step(24); // 6 floats * 4 bytes/float
  
  if (pointCloud.points.empty()) {
    outMsg.set_data("");
    outMsg.set_row_step(0);
    return;
  }
  
  outMsg.set_row_step(pointCloud.points.size() * outMsg.point_step());
  std::string data;
  data.resize(pointCloud.points.size() * outMsg.point_step());
  
  for (size_t i = 0; i < pointCloud.points.size(); ++i) {
    const auto& pt = pointCloud.points[i];
    
    // Transform point to target pose
    gz::math::Vector3d transformedPos = transform.Pos() + transform.Rot().RotateVector(pt.position);
    
    float x = static_cast<float>(transformedPos.X());
    float y = static_cast<float>(transformedPos.Y());
    float z = static_cast<float>(transformedPos.Z());
    float vel = pt.radialVelocity;
    float rcs = pt.rcs;
    float inten = pt.intensity;
    
    unsigned int offset = i * outMsg.point_step();
    memcpy(&data[offset + 0], &x, sizeof(float));
    memcpy(&data[offset + 4], &y, sizeof(float));
    memcpy(&data[offset + 8], &z, sizeof(float));
    memcpy(&data[offset + 12], &vel, sizeof(float));
    memcpy(&data[offset + 16], &rcs, sizeof(float));
    memcpy(&data[offset + 20], &inten, sizeof(float));
  }
  
  outMsg.set_data(data);
}

bool MmWavePointCloudLoader::LoadPointCloudFromFile(const std::string &filename)
{
  try {
    // Open the file
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open()) {
      gzerr << "Failed to open point cloud file: " << filename << std::endl;
      return false;
    }

    // Create a new point cloud
    auto pointCloud = std::make_shared<MmWavePointCloud>();
    
    // Extract the scenario name from the filename
    std::string basename = std::filesystem::path(filename).stem().string();
    pointCloud->name = basename;
    
    // Read header (magic number to verify file format)
    char magic[4];
    file.read(magic, 4);
    if (std::strncmp(magic, "MMPC", 4) != 0) {
      gzerr << "Invalid point cloud file format: " << filename << std::endl;
      return false;
    }
    
    // Read version
    uint32_t version;
    file.read(reinterpret_cast<char*>(&version), sizeof(version));
    if (version != 1) {
      gzerr << "Unsupported point cloud file version: " << version << std::endl;
      return false;
    }
    
    // Read description length and description
    uint32_t descLength;
    file.read(reinterpret_cast<char*>(&descLength), sizeof(descLength));
    
    std::vector<char> descBuffer(descLength);
    file.read(descBuffer.data(), descLength);
    pointCloud->description = std::string(descBuffer.data(), descLength);
    
    // Read sensor parameters
    file.read(reinterpret_cast<char*>(&pointCloud->horizontalFov), sizeof(double));
    file.read(reinterpret_cast<char*>(&pointCloud->verticalFov), sizeof(double));
    file.read(reinterpret_cast<char*>(&pointCloud->horizontalResolution), sizeof(int));
    file.read(reinterpret_cast<char*>(&pointCloud->verticalResolution), sizeof(int));
    
    // Read sensor pose
    double x, y, z, roll, pitch, yaw;
    file.read(reinterpret_cast<char*>(&x), sizeof(double));
    file.read(reinterpret_cast<char*>(&y), sizeof(double));
    file.read(reinterpret_cast<char*>(&z), sizeof(double));
    file.read(reinterpret_cast<char*>(&roll), sizeof(double));
    file.read(reinterpret_cast<char*>(&pitch), sizeof(double));
    file.read(reinterpret_cast<char*>(&yaw), sizeof(double));
    pointCloud->sensorPose = gz::math::Pose3d(x, y, z, roll, pitch, yaw);
    
    // Read number of points
    uint32_t numPoints;
    file.read(reinterpret_cast<char*>(&numPoints), sizeof(numPoints));
    
    // Read points
    for (uint32_t i = 0; i < numPoints; ++i) {
      MmWavePoint point;
      
      // Read position
      double posX, posY, posZ;
      file.read(reinterpret_cast<char*>(&posX), sizeof(double));
      file.read(reinterpret_cast<char*>(&posY), sizeof(double));
      file.read(reinterpret_cast<char*>(&posZ), sizeof(double));
      point.position = gz::math::Vector3d(posX, posY, posZ);
      
      // Read other properties
      file.read(reinterpret_cast<char*>(&point.radialVelocity), sizeof(float));
      file.read(reinterpret_cast<char*>(&point.rcs), sizeof(float));
      file.read(reinterpret_cast<char*>(&point.intensity), sizeof(float));
      file.read(reinterpret_cast<char*>(&point.objectId), sizeof(uint32_t));
      file.read(reinterpret_cast<char*>(&point.classId), sizeof(uint32_t));
      
      pointCloud->points.push_back(point);
    }
    
    // Store the loaded point cloud
    pointClouds[pointCloud->name] = pointCloud;
    
    gzmsg << "Successfully loaded point cloud '" << pointCloud->name 
          << "' with " << pointCloud->points.size() << " points from file: " 
          << filename << std::endl;
    
    return true;
  }
  catch (const std::exception& e) {
    gzerr << "Error loading point cloud file: " << filename 
          << " - " << e.what() << std::endl;
    return false;
  }
}

std::shared_ptr<MmWavePointCloud> MmWavePointCloudLoader::GenerateDefaultPointCloud(const std::string &scenarioName)
{
  auto pointCloud = std::make_shared<MmWavePointCloud>();
  pointCloud->name = scenarioName;
  
  // Set default sensor parameters
  pointCloud->horizontalFov = 1.5708; // 90 degrees
  pointCloud->verticalFov = 0.5236; // 30 degrees
  pointCloud->horizontalResolution = 32;
  pointCloud->verticalResolution = 16;
  pointCloud->sensorPose = gz::math::Pose3d(0, 0, 0, 0, 0, 0); // Origin
  
  int numPoints = 0;
  double rangeNoise = 0.05; // 5cm noise
  
  if (scenarioName == "empty") {
    // Empty room with just floor and walls
    numPoints = 40;
    pointCloud->description = "Empty room with floor and wall points";
    
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
  }
  else if (scenarioName == "room") {
    // Room scene with furniture and people
    numPoints = 100;
    pointCloud->description = "Room scene with furniture and people";
    
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
    
    // Add people points with realistic clustering
    // const int numPeople = 10; // Redeclared - using previous value of 2
    // const int pointsPerPerson = 6; // Redeclared - using previous value of 6
    
    // Create people in small groups or individually
    for (int p = 0; p < numPeople; ++p) {
      // Determine if this person is part of a group
      bool inGroup = (p % 3) != 0; // 2/3 of people are in groups
      
      // Base position for this person
      double baseAngle = gz::math::Rand::DblUniform(0, 2 * M_PI);
      double baseDistance;
      if (inGroup) {
        // Groups tend to be on sidewalks or near buildings
        baseDistance = gz::math::Rand::DblUniform(5.0, 12.0);
        // Adjust angle to be near the previous person if in a group
        if (p > 0 && (p % 3) == 2) {
          baseAngle = gz::math::Rand::DblUniform(
            baseAngle - 0.5, baseAngle + 0.5);
        }
      } else {
        // Individual people can be anywhere
        baseDistance = gz::math::Rand::DblUniform(2.0, 15.0);
      }
      
      double baseX = baseDistance * cos(baseAngle);
      double baseY = baseDistance * sin(baseAngle);
      
      // Generate points for each body part
      for (int i = 0; i < pointsPerPerson; ++i) {
        MmWavePoint point;
        
        // Different points represent different body parts
        double localX, localY, localZ;
        switch (i) {
          case 0: // Head
            localX = gz::math::Rand::DblUniform(-0.1, 0.1);
            localY = gz::math::Rand::DblUniform(-0.1, 0.1);
            localZ = 1.6 + gz::math::Rand::DblUniform(-0.1, 0.1);
            break;
          case 1: // Torso upper
            localX = gz::math::Rand::DblUniform(-0.2, 0.2);
            localY = gz::math::Rand::DblUniform(-0.1, 0.1);
            localZ = 1.3 + gz::math::Rand::DblUniform(-0.1, 0.1);
            break;
          case 2: // Torso lower
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
        
        // Walking speed for people
        double walkingSpeed = gz::math::Rand::DblUniform(1.0, 1.5);
        if (inGroup) walkingSpeed *= 0.8; // Groups walk slower
        
        // Project velocity onto radial direction
        gz::math::Vector3d walkDir(cos(baseAngle + M_PI/2), sin(baseAngle + M_PI/2), 0);
        point.radialVelocity = walkingSpeed * walkDir.Dot(
          point.position.Normalized());
        
        // RCS varies by body part
        point.rcs = (i <= 2) ? 
          gz::math::Rand::DblUniform(0.8, 1.5) :  // Torso/head
          gz::math::Rand::DblUniform(0.3, 0.8);   // Limbs
        
        point.intensity = gz::math::Rand::DblUniform(0.4, 0.7);
        point.objectId = 100 + p;  // Unique ID per person
        point.classId = 2;         // Person class
        
        pointCloud->points.push_back(point);
      }
    }
    
    // Add vehicle points with realistic structure
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
  else if (scenarioName == "bedroom") {
    // Bedroom scene with furniture and possibly a person
    numPoints = 80;
    pointCloud->description = "Bedroom scene with bed, furniture, and possible occupant";
    
    // Room dimensions
    const double roomWidth = 4.0;   // meters
    const double roomLength = 5.0;   // meters
    const double roomHeight = 2.4;   // meters
    
    // Add bed (large furniture piece)
    {
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
  else {
    // Default/fallback case - simple scene with a few objects
    numPoints = 50;
    pointCloud->description = "Default scene with mixed objects";
    
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
  }
  
  // Add noise to all points
  for (auto& point : pointCloud->points) {
    // Add random range noise along the ray direction
    double distance = point.position.Length();
    if (distance > 0) {
      gz::math::Vector3d rayDir = point.position.Normalized();
      double noise = gz::math::Rand::DblNormal(0.0, rangeNoise);
      point.position += rayDir * noise;
    }
  }
  
  gzmsg << "Generated default point cloud '" << scenarioName 
        << "' with " << pointCloud->points.size() << " points." << std::endl;
  
  // Store the generated point cloud
  pointClouds[scenarioName] = pointCloud;
  return pointCloud;
}

} // namespace olympus_sim
