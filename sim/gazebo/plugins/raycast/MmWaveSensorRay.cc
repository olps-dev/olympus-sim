#include "raycast/MmWaveSensorRay.hh"

#include <gz/common/Console.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Name.hh>

using namespace olympus_sim;

MmWaveSensorRay::MmWaveSensorRay()
{
}

void MmWaveSensorRay::SetRayQuery(const gz::rendering::RayQueryPtr &_rayQuery)
{
  this->rayQuery = _rayQuery;
}

void MmWaveSensorRay::CastRays(
    const gz::math::Pose3d &_sensorPose,
    const gz::sim::EntityComponentManager &_ecm,
    gz::msgs::PointCloudPacked &_pointCloudMsg,
    double horizontalFov,
    double horizontalResolution,
    double verticalFov,
    double verticalResolution,
    double minRange,
    double maxRange,
    double noiseMean,
    double noiseStdDev,
    double defaultRCS,
    double minRCS,
    double maxRadialVelocity,
    std::normal_distribution<double> &gaussianNoise)
{
  if (!this->rayQuery)
  {
    gzerr << "[MmWaveSensorRay] Ray query not available, cannot cast rays" << std::endl;
    return;
  }
  
  // Record ray casting attempts for diagnostics
  static int raycastAttempts = 0;
  raycastAttempts++;
  
  gzmsg << "[MmWaveSensorRay] Ray casting attempt #" << raycastAttempts 
        << ", query valid: " << (this->rayQuery ? "yes" : "no") << std::endl;

  static int raycastCounter = 0;
  raycastCounter++;
  
  // Only print detailed logs every 10 frames to avoid spam
  bool detailedLogging = (raycastCounter % 10 == 0);
  
  gzmsg << "[MmWaveSensorRay] Starting ray casting #" << raycastCounter << " with sensor pose: pos="
        << _sensorPose.Pos() << ", rot=" << _sensorPose.Rot().Euler() << std::endl;
        
  if (!this->rayQuery->Scene()) {
    gzerr << "[MmWaveSensorRay] Ray query has null scene! Cannot cast rays." << std::endl;
    return;
  }
  
  // Try to print scene info to verify it's valid
  auto scene = this->rayQuery->Scene();
  gzmsg << "[MmWaveSensorRay] Scene name: '" << scene->Name() 
        << "', node count: " << scene->NodeCount() 
        << ", initialized: " << (scene->IsInitialized() ? "yes" : "no")
        << ", has engine: " << (scene->Engine() != nullptr) << std::endl;
        
  // Log available nodes in the scene for diagnostics
  if (scene->NodeCount() > 0) {
    gzmsg << "[MmWaveSensorRay] Scene contains nodes:" << std::endl;
    for (unsigned int i = 0; i < std::min(5u, scene->NodeCount()); i++) {
      auto node = scene->NodeByIndex(i);
      if (node) {
        gzmsg << "  - Node " << i << ": '" << node->Name() << "'" << std::endl;
      }
    }
    if (scene->NodeCount() > 5) {
      gzmsg << "  - ... and " << (scene->NodeCount() - 5) << " more nodes" << std::endl;
    }
  } else {
    gzwarn << "[MmWaveSensorRay] Scene contains no nodes, ray casting may not detect anything" << std::endl;
  }
        
  // Test if ray query is working properly with a simple ray
  try {
    // Test in multiple directions for more thorough diagnostics
    std::vector<gz::math::Vector3d> testDirs = {
      {1, 0, 0}, {0, 1, 0}, {0, 0, 1},
      {-1, 0, 0}, {0, -1, 0}, {0, 0, -1}
    };
    
    int validRays = 0;
    
    // Start from the world origin and also from the sensor position
    std::vector<gz::math::Vector3d> origins = {
      {0, 0, 0},  // World origin
      _sensorPose.Pos()  // Sensor position
    };
    
    // Try different ray origins and directions
    for (const auto& origin : origins) {
      for (const auto& dir : testDirs) {
        this->rayQuery->SetOrigin(origin);
        this->rayQuery->SetDirection(dir);
        
        // Note: No SetMaxDistance in this version of RayQuery
        // We'll work with default max distance
        
        auto testResult = this->rayQuery->ClosestPoint();
        
        gzmsg << "[MmWaveSensorRay] Test ray from " << origin << " along " << dir
              << ": valid=" << (testResult ? "true" : "false")
              << ", distance=" << testResult.distance << std::endl;
              
        if (testResult) {
          validRays++;
        }
      }
    }
    
    if (validRays == 0) {
      gzerr << "[MmWaveSensorRay] ALL test rays failed, ray casting likely won't work" << std::endl;
    } else {
      gzmsg << "[MmWaveSensorRay] " << validRays << " out of " << (origins.size() * testDirs.size()) 
            << " test rays succeeded" << std::endl;
    }
  } catch (const std::exception& e) {
    gzerr << "[MmWaveSensorRay] Exception during ray casting tests: " << e.what() << std::endl;
  }

  // Calculate horizontal and vertical step sizes
  const double horizontalStepSize = horizontalFov / horizontalResolution;
  const double verticalStepSize = verticalFov / verticalResolution;

  // Number of horizontal and vertical steps
  const int horizontalSteps = static_cast<int>(horizontalFov / horizontalStepSize) + 1;
  const int verticalSteps = static_cast<int>(verticalFov / verticalStepSize) + 1;

  // Storage for point cloud data
  std::vector<float> pointData;
  int pointCount = 0;

  // Cast rays from the sensor's pose
  for (int h = 0; h < horizontalSteps; ++h)
  {
    const double yaw = -horizontalFov / 2.0 + h * horizontalStepSize;
    
    for (int v = 0; v < verticalSteps; ++v)
    {
      const double pitch = -verticalFov / 2.0 + v * verticalStepSize;
      
      // Create ray direction using pitch and yaw
      gz::math::Quaterniond rayRot(0, pitch, yaw);
      gz::math::Vector3d rayDir = rayRot.RotateVector(gz::math::Vector3d(1, 0, 0));
      rayDir.Normalize();
      
      // Transform ray to world frame
      gz::math::Vector3d rayOrigin = _sensorPose.Pos();
      gz::math::Vector3d rayDirWorld = _sensorPose.Rot().RotateVector(rayDir);
      
      // Set up the ray query
      this->rayQuery->SetOrigin(rayOrigin);
      this->rayQuery->SetDirection(rayDirWorld);
      
      // Cast the ray
      gz::rendering::RayQueryResult rayResult = this->rayQuery->ClosestPoint();
      
      // If detailed logging is enabled and this is one of the central rays, print debug info
      if (detailedLogging && h == horizontalSteps/2 && v == verticalSteps/2) {
        gzmsg << "[MmWaveSensorRay] Central ray: origin=" << rayOrigin << ", dir=" << rayDirWorld << std::endl;
        // Check if ray is valid using the boolean operator
        gzmsg << "[MmWaveSensorRay] Ray result: valid=" << (rayResult ? "true" : "false") 
              << ", distance=" << rayResult.distance
              << ", point=" << rayResult.point << std::endl;
      }
      
      // Check if the ray hit something
      if (rayResult && rayResult.distance > minRange && rayResult.distance < maxRange)
      {
        // Calculate the intersection point in world frame
        gz::math::Vector3d intersectionPoint = rayOrigin + rayDirWorld * rayResult.distance;
        
        // Apply noise to the distance
        std::random_device rd;
        std::mt19937 gen(rd());
        double noiseValue = noiseMean + noiseStdDev * gaussianNoise(gen);
        double noisyDistance = rayResult.distance + noiseValue;
        
        // Recalculate the intersection point with noise
        gz::math::Vector3d noisyPoint = rayOrigin + rayDirWorld * noisyDistance;
        
        // Calculate the point in sensor's local frame
        gz::math::Pose3d inversePose = _sensorPose.Inverse();
        gz::math::Vector3d localPoint = inversePose.Rot().RotateVector(noisyPoint - _sensorPose.Pos());
        
        // Calculate radial velocity (Doppler effect)
        double radialVelocity = 0.0;
        // In Gazebo Sim 8, RayQueryResult doesn't have entityId directly
        // We need to use the scene to get the entity ID from the ray result
        gz::sim::Entity hitEntity = gz::sim::kNullEntity;
        
        // Try to get entity ID from the scene graph
        // This is a simplified approach - in a real implementation, you would
        // need to map from rendering entity to simulation entity
        
        if (hitEntity != gz::sim::kNullEntity)
        {
          // Get entity velocity
          auto velComp = _ecm.Component<gz::sim::components::LinearVelocity>(
              hitEntity);
              
          if (velComp)
          {
            const gz::math::Vector3d &targetVel = velComp->Data();
            // Doppler calculation - positive is moving away, negative is moving towards
            radialVelocity = targetVel.Dot(rayDirWorld);
            
            // Cap the velocity
            radialVelocity = std::max(-maxRadialVelocity,
                              std::min(maxRadialVelocity, radialVelocity));
          }
        }
        
        // Estimate radar cross-section
        double rcs = defaultRCS;
        
        // Adjust RCS based on the entity type or distance
        auto nameComp = hitEntity != gz::sim::kNullEntity ? 
            _ecm.Component<gz::sim::components::Name>(hitEntity) : nullptr;
        if (nameComp)
        {
          // Check if the entity name contains specific keywords
          const auto &name = nameComp->Data();
          if (name.find("vehicle") != std::string::npos || 
              name.find("car") != std::string::npos ||
              name.find("truck") != std::string::npos)
          {
            rcs *= 10.0;  // Vehicles have larger RCS
          }
          else if (name.find("person") != std::string::npos ||
                  name.find("human") != std::string::npos)
          {
            rcs *= 0.5;  // People have smaller RCS
          }
          // Ensure RCS is at least the minimum
          rcs = std::max(minRCS, rcs);
        }
        
        // Calculate signal intensity based on distance (radar equation simplification)
        // I ~ 1/r^4 for radar
        double intensity = 1.0 / pow(rayResult.distance, 4);
        
        // Normalize intensity to 0-1 range based on min/max range
        double maxIntensity = 1.0 / pow(minRange, 4);
        double minIntensity = 1.0 / pow(maxRange, 4);
        intensity = (intensity - minIntensity) / (maxIntensity - minIntensity);
        
        // Clamp intensity
        intensity = std::max(0.0, std::min(1.0, intensity));
        
        // Add point data to the buffer (x, y, z, velocity, rcs, intensity)
        pointData.push_back(static_cast<float>(localPoint.X()));
        pointData.push_back(static_cast<float>(localPoint.Y()));
        pointData.push_back(static_cast<float>(localPoint.Z()));
        pointData.push_back(static_cast<float>(radialVelocity));
        pointData.push_back(static_cast<float>(rcs));
        pointData.push_back(static_cast<float>(intensity));
        
        pointCount++;
        
        // Debug individual points periodically
        if (detailedLogging && pointCount <= 3) {
          gzmsg << "[MmWaveSensorRay] Added point " << pointCount << ": (" 
                << localPoint.X() << ", " << localPoint.Y() << ", " << localPoint.Z() 
                << "), world pos: (" << noisyPoint.X() << ", " << noisyPoint.Y() << ", " << noisyPoint.Z() 
                << "), distance: " << rayResult.distance << std::endl;
        }
      }
    }
  }
  
  // Populate the point cloud message
  // We have 6 values per point: x, y, z, velocity, rcs, intensity
  const int fieldsPerPoint = 6;
  // Update the existing pointCount with actual points
  pointCount = pointData.size() / fieldsPerPoint;
  
  // Debug check for data consistency
  if (pointData.size() % fieldsPerPoint != 0) {
    gzwarn << "[MmWaveSensorRay] Inconsistent point data size: " << pointData.size() 
           << " is not divisible by " << fieldsPerPoint << std::endl;
  }
  
  // Set point cloud dimensions
  _pointCloudMsg.set_width(pointCount);
  _pointCloudMsg.set_height(1);
  
  // Each point has 6 fields, each is a float (4 bytes)
  const int bytesPerPoint = fieldsPerPoint * sizeof(float);
  _pointCloudMsg.set_point_step(bytesPerPoint);
  _pointCloudMsg.set_row_step(_pointCloudMsg.point_step() * _pointCloudMsg.width());
  _pointCloudMsg.set_is_dense(true);
  _pointCloudMsg.set_is_bigendian(false);
  
  // Define the fields in the point cloud
  auto addField = [&](const std::string& name, uint32_t offset, gz::msgs::PointCloudPacked_Field_DataType datatype, uint32_t count) {
    auto field = _pointCloudMsg.add_field();
    field->set_name(name);
    field->set_offset(offset);
    field->set_datatype(datatype);
    field->set_count(count);
  };
  
  // Add xyz fields (each is a float)
  addField("x", 0, gz::msgs::PointCloudPacked::Field::FLOAT32, 1);
  addField("y", 4, gz::msgs::PointCloudPacked::Field::FLOAT32, 1);
  addField("z", 8, gz::msgs::PointCloudPacked::Field::FLOAT32, 1);
  addField("velocity", 12, gz::msgs::PointCloudPacked::Field::FLOAT32, 1);
  addField("rcs", 16, gz::msgs::PointCloudPacked::Field::FLOAT32, 1);
  addField("intensity", 20, gz::msgs::PointCloudPacked::Field::FLOAT32, 1);
  
  // Resize data buffer and copy points
  size_t dataSize = pointCount * bytesPerPoint;
  _pointCloudMsg.mutable_data()->resize(dataSize);
  
  if (pointCount > 0) {
    // Use memcpy for more efficient data copy
    memcpy(_pointCloudMsg.mutable_data()->data(), pointData.data(), dataSize);
  }

  // Set up the header with current simulation time
  gz::msgs::Header *header = _pointCloudMsg.mutable_header();
  
  // Use current time 
  auto currentTime = gz::msgs::Time();
  auto now = std::chrono::system_clock::now().time_since_epoch();
  auto seconds = std::chrono::duration_cast<std::chrono::seconds>(now);
  auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(now - seconds);
  
  currentTime.set_sec(seconds.count());
  currentTime.set_nsec(nanoseconds.count());
  
  header->mutable_stamp()->CopyFrom(currentTime);
  
  // Set frame ID
  auto frame = header->add_data();
  frame->set_key("frame_id");
  frame->add_value("mmwave_sensor_link");
  
  // Add debug information
  auto debug = header->add_data();
  debug->set_key("debug");
  debug->add_value("raycast_counter=" + std::to_string(raycastCounter));
  debug->add_value("point_count=" + std::to_string(pointCount));
  
  // Every 10 frames, report the first few points for debugging
  if (pointCount > 0) {
    gzmsg << "[MmWaveSensorRay] Generated point cloud with " << pointCount << " points" << std::endl;
    
    if (detailedLogging) {
      gzmsg << "[MmWaveSensorRay] First 5 points (or fewer if less available):" << std::endl;
      int pointsToPrint = std::min(5, pointCount);
      for (int i = 0; i < pointsToPrint; ++i) {
        int idx = i * fieldsPerPoint;
        if (idx + fieldsPerPoint - 1 < static_cast<int>(pointData.size())) {
          gzmsg << "  Point " << i << ": (" 
                << pointData[idx] << ", " 
                << pointData[idx+1] << ", " 
                << pointData[idx+2] << "), velocity=" 
                << pointData[idx+3] << ", rcs=" 
                << pointData[idx+4] << ", intensity=" 
                << pointData[idx+5] << std::endl;
        }
      }
    }
  } else {
    gzwarn << "[MmWaveSensorRay] NO POINTS generated in ray casting attempt #" << raycastCounter << "!" << std::endl;
    
    // Debug information about the scene when no points are found
    auto scene = this->rayQuery->Scene();
    if (scene) {
      gzmsg << "[MmWaveSensorRay] Scene status: name='" << scene->Name()
            << "', initialized=" << (scene->IsInitialized() ? "true" : "false")
            << ", node count=" << scene->NodeCount() << std::endl;
    }
  }

  gzmsg << "[MmWaveSensorRay] Ray casting complete, generated " 
        << pointCount << " points" << std::endl;
        
  // Log detailed statistics about the ray casting
  if (pointCount > 0) {
    gzmsg << "[MmWaveSensorRay] Point cloud statistics:" << std::endl;
    gzmsg << "  - Horizontal steps: " << horizontalSteps << std::endl;
    gzmsg << "  - Vertical steps: " << verticalSteps << std::endl;
    gzmsg << "  - Total rays cast: " << horizontalSteps * verticalSteps << std::endl;
    gzmsg << "  - Hit percentage: " << (pointCount * 100.0) / (horizontalSteps * verticalSteps) << "%" << std::endl;
    
    // Log some sample points (first 3 points)
    int sampleCount = std::min(3, pointCount);
    for (int i = 0; i < sampleCount; ++i) {
      int offset = i * 6; // 6 values per point
      gzmsg << "  - Sample point " << i << ": (" 
            << pointData[offset] << ", " 
            << pointData[offset+1] << ", " 
            << pointData[offset+2] << "), velocity=" 
            << pointData[offset+3] << ", rcs=" 
            << pointData[offset+4] << ", intensity=" 
            << pointData[offset+5] << std::endl;
    }
  } else {
    gzmsg << "[MmWaveSensorRay] No points generated. Check if there are objects in the sensor's field of view." << std::endl;
    gzmsg << "  - Horizontal FOV: " << horizontalFov << " rad, steps: " << horizontalSteps << std::endl;
    gzmsg << "  - Vertical FOV: " << verticalFov << " rad, steps: " << verticalSteps << std::endl;
    gzmsg << "  - Range: " << minRange << " to " << maxRange << " meters" << std::endl;
  }
}
