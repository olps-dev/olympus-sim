#include "MmWavePointCloudLoader.hh"
#include "MmWavePointCloudScenarioGenerators.hh"

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
  
  // Try to load point clouds from directory
  bool foundAny = LoadPointCloudsFromDirectory(pointCloudDirectory);

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
  // Initialize message header
  SetupPointCloudMessageHeader(pointCloud, outMsg);
  
  // Handle empty point clouds
  if (pointCloud.points.empty()) {
    outMsg.set_data("");
    outMsg.set_row_step(0);
    return;
  }
  
  // Calculate transform and convert point data
  SetupPointCloudData(pointCloud, targetPose, outMsg);
}

void MmWavePointCloudLoader::SetupPointCloudMessageHeader(
    const MmWavePointCloud &pointCloud,
    gz::msgs::PointCloudPacked &outMsg)
{
  // Set point cloud message header properties
  outMsg.set_width(pointCloud.points.size());
  outMsg.set_height(1);  // Unorganized point cloud (list of points)
  outMsg.set_is_dense(true);  // No invalid points
  
  // Define field structure: x, y, z, velocity, rcs, intensity
  // Each point has 6 float values (24 bytes total)
  outMsg.set_point_step(24);  // 6 floats * 4 bytes/float

  // Define fields for the point cloud
  gz::msgs::PointCloudPacked::Field *field;
  
  // X position field
  field = outMsg.add_field();
  field->set_name("x");
  field->set_offset(0);
  field->set_datatype(gz::msgs::PointCloudPacked::Field::FLOAT32);
  field->set_count(1);
  
  // Y position field
  field = outMsg.add_field();
  field->set_name("y");
  field->set_offset(4);
  field->set_datatype(gz::msgs::PointCloudPacked::Field::FLOAT32);
  field->set_count(1);
  
  // Z position field
  field = outMsg.add_field();
  field->set_name("z");
  field->set_offset(8);
  field->set_datatype(gz::msgs::PointCloudPacked::Field::FLOAT32);
  field->set_count(1);
  
  // Velocity field
  field = outMsg.add_field();
  field->set_name("velocity");
  field->set_offset(12);
  field->set_datatype(gz::msgs::PointCloudPacked::Field::FLOAT32);
  field->set_count(1);
  
  // RCS (radar cross-section) field
  field = outMsg.add_field();
  field->set_name("rcs");
  field->set_offset(16);
  field->set_datatype(gz::msgs::PointCloudPacked::Field::FLOAT32);
  field->set_count(1);
  
  // Intensity field
  field = outMsg.add_field();
  field->set_name("intensity");
  field->set_offset(20);
  field->set_datatype(gz::msgs::PointCloudPacked::Field::FLOAT32);
  field->set_count(1);
}

void MmWavePointCloudLoader::SetupPointCloudData(
    const MmWavePointCloud &pointCloud,
    const gz::math::Pose3d &targetPose,
    gz::msgs::PointCloudPacked &outMsg)
{
  // Calculate the transform from the point cloud's sensor pose to the target pose
  gz::math::Pose3d transform = targetPose.Inverse() * pointCloud.sensorPose;
  
  // Set up data buffer for all points
  outMsg.set_row_step(pointCloud.points.size() * outMsg.point_step());
  std::string data;
  data.resize(pointCloud.points.size() * outMsg.point_step());
  
  // Process each point
  for (size_t i = 0; i < pointCloud.points.size(); ++i) {
    const auto& pt = pointCloud.points[i];
    unsigned int offset = i * outMsg.point_step();
    
    // Add the transformed point to the data buffer
    AddTransformedPointToBuffer(pt, transform, data, offset);
  }
  
  // Set the data in the message
  outMsg.set_data(data);
}

void MmWavePointCloudLoader::AddTransformedPointToBuffer(
    const MmWavePoint &point,
    const gz::math::Pose3d &transform,
    std::string &data,
    unsigned int offset)
{
  // Transform point to target pose
  gz::math::Vector3d transformedPos = transform.Pos() + transform.Rot().RotateVector(point.position);
  
  // Convert to float values
  float x = static_cast<float>(transformedPos.X());
  float y = static_cast<float>(transformedPos.Y());
  float z = static_cast<float>(transformedPos.Z());
  float vel = point.radialVelocity;
  float rcs = point.rcs;
  float inten = point.intensity;
  
  // Copy data to buffer
  memcpy(&data[offset + 0], &x, sizeof(float));
  memcpy(&data[offset + 4], &y, sizeof(float));
  memcpy(&data[offset + 8], &z, sizeof(float));
  memcpy(&data[offset + 12], &vel, sizeof(float));
  memcpy(&data[offset + 16], &rcs, sizeof(float));
  memcpy(&data[offset + 20], &inten, sizeof(float));
}

bool MmWavePointCloudLoader::LoadPointCloudsFromDirectory(const std::string &directory)
{
  if (!std::filesystem::exists(directory) || !std::filesystem::is_directory(directory)) {
    gzerr << "Invalid point cloud directory: " << directory << std::endl;
    return false;
  }

  bool foundAny = false;
  try {
    gzmsg << "Scanning for point cloud files in: " << directory << std::endl;
    
    // Iterate through directory and load all .mmpc files
    for (const auto & entry : std::filesystem::directory_iterator(directory)) {
      if (entry.is_regular_file() && entry.path().extension() == ".mmpc") {
        gzdbg << "Found point cloud file: " << entry.path().string() << std::endl;
        if (LoadPointCloudFromFile(entry.path().string())) {
          foundAny = true;
        }
      }
    }
    
    // Report results
    if (foundAny) {
      gzmsg << "Successfully loaded " << pointClouds.size() 
            << " point clouds from directory." << std::endl;
    } else {
      gzwarn << "No valid point cloud files found in directory: " << directory << std::endl;
    }
  } catch (const std::exception& e) {
    gzerr << "Error scanning point cloud directory: " << directory 
          << " - " << e.what() << std::endl;
  }
  
  return foundAny;
}

bool MmWavePointCloudLoader::LoadPointCloudFromFile(const std::string &filename)
{
  try {
    // Check if file exists
    if (!std::filesystem::exists(filename)) {
      gzerr << "Point cloud file does not exist: " << filename << std::endl;
      return false;
    }
    
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
    
    // Read and validate file format
    if (!ReadFileHeader(file, filename)) {
      return false;
    }
    
    // Read point cloud metadata
    if (!ReadPointCloudMetadata(file, pointCloud)) {
      gzerr << "Failed to read point cloud metadata from: " << filename << std::endl;
      return false;
    }
    
    // Read point cloud data
    if (!ReadPointCloudData(file, pointCloud)) {
      gzerr << "Failed to read point cloud data from: " << filename << std::endl;
      return false;
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

bool MmWavePointCloudLoader::ReadFileHeader(std::ifstream &file, const std::string &filename)
{
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
  
  return true;
}

bool MmWavePointCloudLoader::ReadPointCloudMetadata(
    std::ifstream &file, 
    std::shared_ptr<MmWavePointCloud> pointCloud)
{
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
  
  return !file.fail();
}

bool MmWavePointCloudLoader::ReadPointCloudData(
    std::ifstream &file, 
    std::shared_ptr<MmWavePointCloud> pointCloud)
{
  // Read number of points
  uint32_t numPoints;
  file.read(reinterpret_cast<char*>(&numPoints), sizeof(numPoints));
  
  // Reserve space for efficiency
  pointCloud->points.reserve(numPoints);
  
  // Read points
  for (uint32_t i = 0; i < numPoints; ++i) {
    if (!ReadSinglePoint(file, pointCloud)) {
      gzerr << "Failed to read point " << i << " of " << numPoints << std::endl;
      return false;
    }
  }
  
  return !file.fail() && pointCloud->points.size() == numPoints;
}

bool MmWavePointCloudLoader::ReadSinglePoint(
    std::ifstream &file, 
    std::shared_ptr<MmWavePointCloud> pointCloud)
{
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
  
  if (!file.fail()) {
    pointCloud->points.push_back(point);
    return true;
  }
  
  return false;
}

std::shared_ptr<MmWavePointCloud> MmWavePointCloudLoader::GenerateDefaultPointCloud(const std::string &scenarioName)
{
  // Use the factory to create the appropriate generator
  auto generator = MmWavePointCloudGeneratorFactory::CreateGenerator(scenarioName);
  
  // Generate the point cloud
  auto pointCloud = generator->Generate(scenarioName);
  
  // Store the generated point cloud
  if (pointCloud) {
    pointClouds[scenarioName] = pointCloud;
    gzmsg << "Generated default point cloud '" << scenarioName 
          << "' with " << pointCloud->points.size() << " points." << std::endl;
  }
  
  return pointCloud;
}

} // namespace olympus_sim
