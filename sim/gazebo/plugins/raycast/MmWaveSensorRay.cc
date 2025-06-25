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
    gzwarn << "[MmWaveSensorRay] Ray query not available, cannot cast rays" << std::endl;
    return;
  }

  gzdbg << "[MmWaveSensorRay] Starting ray casting with sensor pose: pos="
        << _sensorPose.Pos() << ", rot=" << _sensorPose.Rot().Euler() << std::endl;

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
      }
    }
  }
  
  // Set point cloud message data
  _pointCloudMsg.set_width(pointCount);
  _pointCloudMsg.set_height(1);
  _pointCloudMsg.set_row_step(_pointCloudMsg.point_step() * pointCount);
  
  // Convert point data to binary format
  if (pointCount > 0)
  {
    const size_t dataSize = pointCount * _pointCloudMsg.point_step();
    _pointCloudMsg.mutable_data()->resize(dataSize);
    memcpy(_pointCloudMsg.mutable_data()->data(), pointData.data(), dataSize);
  }
  
  gzdbg << "[MmWaveSensorRay] Ray casting complete, generated " 
        << pointCount << " points" << std::endl;
}
