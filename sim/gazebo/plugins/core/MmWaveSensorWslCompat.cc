#include "core/MmWaveSensorWslCompat.hh"

// Component headers already included by MmWaveSensorWslCompat.hh
#include <gz/common/Console.hh>
#include <gz/math/Rand.hh>

#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <cstring> // For memcpy

namespace olympus_sim
{
namespace wsl_compat
{

double CalculateRCS(
    const gz::sim::Entity &_entity,
    const gz::sim::EntityComponentManager &_ecm,
    double defaultRCSValue,
    double minRCSValue)
{
  // Note: RadarCrossSection component doesn't exist in current Gazebo version
  // We'll use name-based heuristics for RCS calculation

  auto nameComp = _ecm.Component<gz::sim::components::Name>(_entity);
  if (nameComp)
  {
    const auto& name = nameComp->Data();
    if (name.find("person") != std::string::npos || name.find("actor") != std::string::npos)
    {
      return std::max(minRCSValue, 1.0); // Typical RCS for a person
    }
    else if (name.find("car") != std::string::npos || name.find("vehicle") != std::string::npos)
    {
      return std::max(minRCSValue, 10.0); // Typical RCS for a car
    }
    else if (name.find("building") != std::string::npos || name.find("house") != std::string::npos)
    {
      return std::max(minRCSValue, 100.0); // RCS for a building
    }
  }
  return std::max(minRCSValue, defaultRCSValue);
}

double CalculateRadialVelocity(
    const gz::sim::Entity &_targetEntity,
    const gz::sim::Entity &_sensorEntity,
    const gz::math::Pose3d & /*_sensorPose*/, // _sensorPose currently unused as velocities are in world frame
    const gz::math::Vector3d &_rayDirection,
    const gz::sim::EntityComponentManager &_ecm,
    double maxRadialVelocityValue)
{
  gz::math::Vector3d targetVel = gz::math::Vector3d::Zero;
  auto targetLinVelComp = _ecm.Component<gz::sim::components::WorldLinearVelocity>(_targetEntity);
  if (targetLinVelComp)
  {
    targetVel = targetLinVelComp->Data();
  }

  gz::math::Vector3d sensorVel = gz::math::Vector3d::Zero;
  auto sensorLinVelComp = _ecm.Component<gz::sim::components::WorldLinearVelocity>(_sensorEntity);
  if (sensorLinVelComp)
  {
    sensorVel = sensorLinVelComp->Data();
  }

  gz::math::Vector3d relativeVel = targetVel - sensorVel;
  double radialVelocity = relativeVel.Dot(_rayDirection.Normalized());

  return std::max(-maxRadialVelocityValue, std::min(radialVelocity, maxRadialVelocityValue));
}

void UpdateSimplifiedWorldModel(
    const gz::sim::Entity &_sensorEntity,
    const gz::sim::EntityComponentManager &_ecm,
    std::map<gz::sim::Entity, std::pair<gz::math::Pose3d, gz::math::Vector3d>> &simplifiedWorldModel,
    bool &wslWarningShown)
{
  simplifiedWorldModel.clear();
  size_t entitiesAdded = 0;

  // Iterate over all models in the world that have a pose
  _ecm.Each<gz::sim::components::Model, gz::sim::components::Name, gz::sim::components::WorldPose>(
    [&](const gz::sim::Entity &_entity,
        const gz::sim::components::Model * /*_model*/,
        const gz::sim::components::Name *_name,
        const gz::sim::components::WorldPose *_worldPose) -> bool
    {
      // Skip the sensor's own parent model entity
      auto parent = _ecm.ParentEntity(_sensorEntity);
      if (_entity == _sensorEntity || (parent != gz::sim::kNullEntity && _entity == parent))
      {
        return true; // continue iteration
      }

      gzmsg << "[MmWaveSensorPlugin::UpdateSimplifiedWorldModel] Found model: " << _name->Data() << std::endl;

      gz::math::Pose3d pose = _worldPose->Data();
      gz::math::Vector3d velocity = gz::math::Vector3d::Zero;
      auto linVelComp = _ecm.Component<gz::sim::components::WorldLinearVelocity>(_entity);
      if (linVelComp)
      {
        velocity = linVelComp->Data();
      }
      simplifiedWorldModel[_entity] = std::make_pair(pose, velocity);
      entitiesAdded++;
      return true; // continue iteration
    });

  if (entitiesAdded == 0 && !wslWarningShown)
  {
    gzwarn << "[MmWaveSensorPlugin::UpdateSimplifiedWorldModel] WARNING: No entities (excluding sensor) found for simplified world model! "
           << "Sensor will not detect anything in WSL compatibility mode." << std::endl;
    wslWarningShown = true;
  } else if (entitiesAdded > 0) {
    wslWarningShown = false; // Reset warning if entities are found
  }
}

void GenerateSimulatedData(
    const gz::sim::Entity &_sensorEntity,
    const gz::math::Pose3d &_sensorPose,
    const gz::sim::EntityComponentManager &_ecm,
    gz::msgs::PointCloudPacked &_pointCloudMsg,
    const std::map<gz::sim::Entity, std::pair<gz::math::Pose3d, gz::math::Vector3d>> &simplifiedWorldModel,
    double horizontalFov,
    int horizontalResolution,
    double verticalFov,
    int verticalResolution,
    double minRange,
    double maxRange,
    double noiseMean,
    double noiseStdDev,
    double defaultRCSValue,
    double minRCSValue,
    double maxRadialVelocityValue)
{
  gzmsg << "[MmWaveSensorPlugin::GenerateSimulatedData] Starting simulated data generation (WSL compat mode)" << std::endl;
  gzmsg << "[MmWaveSensorPlugin::GenerateSimulatedData] Number of entities in simplified world model: " << simplifiedWorldModel.size() << std::endl;
  
  // Print all detected entities
  for (const auto& [entityId, poseVel] : simplifiedWorldModel) {
    auto nameComp = _ecm.Component<gz::sim::components::Name>(entityId);
    if (nameComp) {
      gzmsg << "[MmWaveSensorPlugin::GenerateSimulatedData] Entity: " << entityId 
            << " Name: " << nameComp->Data() 
            << " Pose: " << poseVel.first << std::endl;
    }
  }

  if (simplifiedWorldModel.empty()) {
    gzdbg << "[MmWaveSensorPlugin::GenerateSimulatedData] No objects in world model (WSL compat mode)" << std::endl;
    _pointCloudMsg.set_width(0);
    _pointCloudMsg.set_height(0);
    _pointCloudMsg.set_row_step(0);
    _pointCloudMsg.set_data("");
    _pointCloudMsg.set_is_dense(true);
    return;
  }
  
  std::vector<gz::math::Vector3d> rayDirections;
  const double horizontalAngleStep = horizontalResolution > 1 ? horizontalFov / (horizontalResolution - 1) : 0.0;
  const double verticalAngleStep = verticalResolution > 1 ? verticalFov / (verticalResolution - 1) : 0.0;
  
  for (int v = 0; v < verticalResolution; ++v) {
    double vertAngle = verticalResolution > 1 ? -verticalFov/2.0 + v * verticalAngleStep : 0.0;
    for (int h = 0; h < horizontalResolution; ++h) {
      double horizAngle = horizontalResolution > 1 ? -horizontalFov/2.0 + h * horizontalAngleStep : 0.0;
      double cosV = cos(vertAngle);
      double sinV = sin(vertAngle);
      double cosH = cos(horizAngle);
      double sinH = sin(horizAngle);
      gz::math::Vector3d dir(cosV * cosH, cosV * sinH, sinV);
      gz::math::Vector3d worldDir = _sensorPose.Rot().RotateVector(dir);
      rayDirections.push_back(worldDir.Normalized());
    }
  }
  
  std::vector<gz::math::Vector3d> hitPoints;
  std::vector<double> velocities;
  std::vector<double> intensities;
  std::vector<double> rcsValues;
  
  gzmsg << "[MmWaveSensorPlugin::GenerateSimulatedData] Checking " 
        << simplifiedWorldModel.size() << " objects against " 
        << rayDirections.size() << " rays (WSL compat mode)" << std::endl;

  for (const auto& [entityId, poseVel] : simplifiedWorldModel) {
    const auto& objectPose = poseVel.first;
    double objectRadius = 1.0; 
    auto nameComp = _ecm.Component<gz::sim::components::Name>(entityId);
    if (nameComp) {
      const auto& name = nameComp->Data();
      if (name.find("car") != std::string::npos || name.find("vehicle") != std::string::npos) {
        objectRadius = 2.0;
      } else if (name.find("building") != std::string::npos || name.find("house") != std::string::npos) {
        objectRadius = 5.0;
      }
    }
    
    double objectRCS = wsl_compat::CalculateRCS(entityId, _ecm, defaultRCSValue, minRCSValue);
    
    gz::math::Vector3d toObject = objectPose.Pos() - _sensorPose.Pos();
    double distToObjectCenter = toObject.Length();
    
    if (distToObjectCenter - objectRadius > maxRange) { // Optimization: if even closest point of sphere is too far
      continue;
    }
    
    for (const auto& rayDir : rayDirections) {
      double rayDotToObject = toObject.Dot(rayDir);
      if (rayDotToObject < 0) { // Object is behind the ray's origin
        continue;
      }
      
      gz::math::Vector3d pointOnRayClosestToObject = _sensorPose.Pos() + rayDir * rayDotToObject;
      double perpDistToCenter = (pointOnRayClosestToObject - objectPose.Pos()).Length();
      
      if (perpDistToCenter <= objectRadius) { // Ray intersects the sphere
        double distToPerpIntersect = sqrt(std::max(0.0, objectRadius*objectRadius - perpDistToCenter*perpDistToCenter));
        double distToActualIntersection = rayDotToObject - distToPerpIntersect;
        
        if (distToActualIntersection < minRange || distToActualIntersection > maxRange) {
          continue;
        }
        
        gz::math::Vector3d hitPoint = _sensorPose.Pos() + rayDir * distToActualIntersection;
        double noiseVal = gz::math::Rand::DblNormal(noiseMean, noiseStdDev);
        hitPoint += rayDir * noiseVal; // Apply noise along the ray direction
        
        double radialVelocity = wsl_compat::CalculateRadialVelocity(entityId, _sensorEntity, _sensorPose, rayDir, _ecm, maxRadialVelocityValue);
        
        // Simplified intensity: RCS / R^4, scaled
        double intensity = objectRCS / (distToActualIntersection * distToActualIntersection * distToActualIntersection * distToActualIntersection);
        intensity = std::min(1.0, intensity * 1e4); // Adjust scaling factor as needed
        
        hitPoints.push_back(hitPoint);
        velocities.push_back(radialVelocity);
        intensities.push_back(intensity);
        rcsValues.push_back(objectRCS);
      }
    }
  }
  
  _pointCloudMsg.set_width(hitPoints.size());
  _pointCloudMsg.set_height(1);
  _pointCloudMsg.set_is_dense(true); // Assuming all points are valid
  _pointCloudMsg.set_point_step(24); // 6 floats * 4 bytes/float

  if (hitPoints.empty()) {
    _pointCloudMsg.set_data("");
    _pointCloudMsg.set_row_step(0);
  } else {
    _pointCloudMsg.set_row_step(hitPoints.size() * _pointCloudMsg.point_step());
    std::string data;
    data.resize(hitPoints.size() * _pointCloudMsg.point_step());
    for (size_t i = 0; i < hitPoints.size(); ++i) {
      const auto& pt = hitPoints[i];
      float x = static_cast<float>(pt.X());
      float y = static_cast<float>(pt.Y());
      float z = static_cast<float>(pt.Z());
      float vel = static_cast<float>(velocities[i]);
      float rcs = static_cast<float>(rcsValues[i]);
      float inten = static_cast<float>(intensities[i]);
      unsigned int offset = i * _pointCloudMsg.point_step();
      memcpy(&data[offset + 0], &x, sizeof(float));
      memcpy(&data[offset + 4], &y, sizeof(float));
      memcpy(&data[offset + 8], &z, sizeof(float));
      memcpy(&data[offset + 12], &vel, sizeof(float));
      memcpy(&data[offset + 16], &rcs, sizeof(float));
      memcpy(&data[offset + 20], &inten, sizeof(float));
    }
    _pointCloudMsg.set_data(data);
  }
  
  gzmsg << "[MmWaveSensorPlugin::GenerateSimulatedData] Generated " 
        << hitPoints.size() << " points (WSL compat mode)" << std::endl;
}

} // namespace wsl_compat
} // namespace olympus_sim
