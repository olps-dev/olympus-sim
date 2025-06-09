#include "MmWaveSensorPlugin.hh"

#include <gz/plugin/Register.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/components/LinearVelocity.hh> // For Doppler velocity
#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/RayQuery.hh>
#include <gz/math/Rand.hh> // For noise simulation
#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <cstring>
#include <cmath>

using namespace olympus_sim;
using namespace gz;
using namespace gz::sim;

MmWaveSensorPlugin::MmWaveSensorPlugin()
{
  // Constructor can be used for initial member variable setup if needed
}

void MmWaveSensorPlugin::Configure(const Entity &_entity,
                                   const std::shared_ptr<const sdf::Element> &_sdf,
                                   EntityComponentManager &_ecm,
                                   EventManager & /*_eventMgr*/)
{
  this->entity = _entity;
  this->sdfConfig = _sdf;

  // Get plugin name
  std::string pluginName = "MmWaveSensorPlugin";
  if (_sdf->HasAttribute("name"))
  {
    pluginName = _sdf->Get<std::string>("name");
  }
  gzmsg << "[" << pluginName << "] Configuring plugin for entity ["
        << _ecm.Component<components::Name>(this->entity)->Data() << "]\n";

  // Load parameters from SDF
  if (_sdf->HasElement("topic"))
  {
    this->topicName = _sdf->Get<std::string>("topic");
  }
  if (_sdf->HasElement("update_rate"))
  {
    this->updateRate = _sdf->Get<double>("update_rate");
    if (this->updateRate > 0)
    {
        this->updatePeriod = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(1.0 / this->updateRate));
    }
  }
  if (_sdf->HasElement("horizontal_fov"))
  {
    this->horizontalFov = _sdf->Get<double>("horizontal_fov");
  }
  if (_sdf->HasElement("vertical_fov"))
  {
    this->verticalFov = _sdf->Get<double>("vertical_fov");
  }
  if (_sdf->HasElement("min_range"))
  {
    this->minRange = _sdf->Get<double>("min_range");
  }
  if (_sdf->HasElement("max_range"))
  {
    this->maxRange = _sdf->Get<double>("max_range");
  }
  if (_sdf->HasElement("horizontal_resolution"))
  {
    this->horizontalResolution = _sdf->Get<int>("horizontal_resolution");
  }
  if (_sdf->HasElement("vertical_resolution"))
  {
    this->verticalResolution = _sdf->Get<int>("vertical_resolution");
  }
  
  // Load additional parameters
  if (_sdf->HasElement("noise_mean"))
  {
    this->noiseMean = _sdf->Get<double>("noise_mean");
  }
  if (_sdf->HasElement("noise_stddev"))
  {
    this->noiseStdDev = _sdf->Get<double>("noise_stddev");
  }
  if (_sdf->HasElement("default_rcs"))
  {
    this->defaultRCS = _sdf->Get<double>("default_rcs");
  }
  if (_sdf->HasElement("min_rcs"))
  {
    this->minRCS = _sdf->Get<double>("min_rcs");
  }
  if (_sdf->HasElement("max_radial_velocity"))
  {
    this->maxRadialVelocity = _sdf->Get<double>("max_radial_velocity");
  }
  if (_sdf->HasElement("visualize"))
  {
    this->visualize = _sdf->Get<bool>("visualize");
  }

  gzmsg << "[" << pluginName << "] Parameters loaded: \n"
        << "  Topic: " << this->topicName << "\n"
        << "  Update Rate: " << this->updateRate << " Hz\n"
        << "  Horizontal FOV: " << this->horizontalFov << " rad\n"
        << "  Vertical FOV: " << this->verticalFov << " rad\n"
        << "  Min Range: " << this->minRange << " m\n"
        << "  Max Range: " << this->maxRange << " m\n"
        << "  Horizontal Resolution: " << this->horizontalResolution << "\n"
        << "  Vertical Resolution: " << this->verticalResolution << "\n"
        << "  Noise Mean: " << this->noiseMean << " m\n"
        << "  Noise StdDev: " << this->noiseStdDev << " m\n"
        << "  Default RCS: " << this->defaultRCS << " m^2\n"
        << "  Min RCS: " << this->minRCS << " m^2\n"
        << "  Max Radial Velocity: " << this->maxRadialVelocity << " m/s\n"
        << "  Visualize: " << (this->visualize ? "true" : "false") << "\n";

  // Initialize Gazebo transport publisher
  this->pointCloudPublisher = this->node.Advertise<msgs::PointCloudPacked>(this->topicName);
  if (!this->pointCloudPublisher)
  {
    gzerr << "[" << pluginName << "] Failed to create publisher on topic [" << this->topicName << "].\n";
  }
}

//////////////////////////////////////////////////
// Helper methods implementation
//////////////////////////////////////////////////

gz::rendering::ScenePtr MmWaveSensorPlugin::Scene() const
{
  // For now, just return nullptr as we're not using actual ray casting
  // This is a placeholder until we can properly integrate with the rendering system
  return nullptr;
}

//////////////////////////////////////////////////
double MmWaveSensorPlugin::CalculateRCS(const gz::sim::Entity &_entity,
                                       const gz::sim::EntityComponentManager &_ecm) const
{
  // In a real implementation, we would look for a custom RCS component on the entity
  // or calculate it based on the entity's material properties and geometry.
  // For now, we'll return a default value.
  
  // Check if entity has a name component (for debugging)
  auto nameComp = _ecm.Component<components::Name>(_entity);
  if (nameComp)
  {
    // In the future, we could use the name to look up RCS values from a database
    // or use pattern matching to assign different RCS values to different types of objects
    const std::string &name = nameComp->Data();
    
    // Example: assign higher RCS to objects with "metal" in their name
    if (name.find("metal") != std::string::npos)
      return 2.0 * this->defaultRCS;
  }
  
  // Default RCS value
  return this->defaultRCS;
}

//////////////////////////////////////////////////
double MmWaveSensorPlugin::CalculateRadialVelocity(const gz::sim::Entity &_entity,
                                                 const gz::math::Vector3d &_rayDirection,
                                                 const gz::sim::EntityComponentManager &_ecm) const
{
  // Get the entity's linear velocity if available
  auto velComp = _ecm.Component<components::LinearVelocity>(_entity);
  if (!velComp)
    return 0.0;  // No velocity component, assume stationary
    
  // Get the velocity vector
  const auto &velocity = velComp->Data();
  
  // Calculate radial velocity (projection of velocity onto ray direction)
  // Positive value means object is moving away from sensor
  // Negative value means object is moving toward sensor
  double radialVel = velocity.Dot(_rayDirection);
  
  // Clamp to maximum range
  return std::max(-this->maxRadialVelocity, std::min(radialVel, this->maxRadialVelocity));
}

//////////////////////////////////////////////////
void MmWaveSensorPlugin::CastRays(const gz::math::Pose3d &_sensorPose,
                                 const gz::sim::EntityComponentManager &_ecm,
                                 gz::msgs::PointCloudPacked &_pointCloudMsg)
{
  
  // Calculate angular step sizes based on FOV and resolution
  double horzStep = this->horizontalFov / static_cast<double>(this->horizontalResolution);
  double vertStep = this->verticalFov / static_cast<double>(this->verticalResolution);
  
  // Calculate starting angles (centered around sensor forward direction)
  double horzStart = -this->horizontalFov / 2.0;
  double vertStart = -this->verticalFov / 2.0;
  
  // Vector to store detected points
  std::vector<float> points;
  
  // Count of valid detections
  int validDetections = 0;
  
  // Cast rays for each pixel in the resolution grid
  for (int h = 0; h < this->horizontalResolution; ++h)
  {
    double horzAngle = horzStart + h * horzStep;
    
    for (int v = 0; v < this->verticalResolution; ++v)
    {
      double vertAngle = vertStart + v * vertStep;
      
      // Calculate ray direction in sensor local frame
      // Forward is X, right is Y, up is Z
      math::Vector3d localDir(
        std::cos(vertAngle) * std::cos(horzAngle),  // X
        std::cos(vertAngle) * std::sin(horzAngle),  // Y
        std::sin(vertAngle)                         // Z
      );
      
      // Transform direction to world frame
      math::Vector3d worldDir = _sensorPose.Rot() * localDir;
      worldDir.Normalize();
      
      // For now, just simulate some points at random distances
      // This is a placeholder until we can properly integrate with the rendering system
      if (math::Rand::DblUniform(0.0, 1.0) < 0.1)  // 10% chance of detection
      {
        // Generate a random distance within range
        double distance = math::Rand::DblUniform(this->minRange, this->maxRange);
        
        // Get the intersection point in world coordinates
        math::Vector3d hitPoint = _sensorPose.Pos() + worldDir * distance;
        
        // Add noise to the distance measurement
        if (this->noiseStdDev > 0)
        {
          double noise = math::Rand::DblNormal(this->noiseMean, this->noiseStdDev);
          distance += noise;
          
          // Recalculate hit point with noise
          hitPoint = _sensorPose.Pos() + worldDir * distance;
        }
        
        // Use default RCS
        double rcs = this->defaultRCS;
        
        // Skip if RCS is below minimum detectable
        if (rcs < this->minRCS)
          continue;
        
        // Default radial velocity
        double radialVelocity = 0.0;
        
        // Calculate signal intensity based on radar equation
        // I ~ 1/r^4 for radar (two-way propagation)
        double intensity = rcs / (distance * distance * distance * distance);
        
        // Normalize intensity to [0,1] range
        intensity = std::min(1.0, intensity);
        
        // Add point data
        points.push_back(static_cast<float>(hitPoint.X()));
        points.push_back(static_cast<float>(hitPoint.Y()));
        points.push_back(static_cast<float>(hitPoint.Z()));
        points.push_back(static_cast<float>(radialVelocity));
        points.push_back(static_cast<float>(rcs));
        points.push_back(static_cast<float>(intensity));
        
        validDetections++;
      }
    }
  }
  
  // Update point cloud message with detected points
  _pointCloudMsg.set_width(validDetections);
  _pointCloudMsg.set_height(1);  // unstructured point cloud
  _pointCloudMsg.set_row_step(validDetections * _pointCloudMsg.point_step());
  
  // Set the point data
  if (validDetections > 0)
  {
    std::string data;
    data.resize(points.size() * sizeof(float));
    memcpy(&data[0], points.data(), points.size() * sizeof(float));
    _pointCloudMsg.set_data(data);
  }
}

//////////////////////////////////////////////////
// PostUpdate implementation
//////////////////////////////////////////////////

void MmWaveSensorPlugin::PostUpdate(const UpdateInfo &_info,
                                    const EntityComponentManager &_ecm)
{
  // Throttle updates
  if (_info.simTime < this->lastUpdateTime + this->updatePeriod && _info.simTime > std::chrono::seconds(0)) {
      return;
  }
  this->lastUpdateTime = _info.simTime;

  // Get sensor pose
  auto poseComp = _ecm.Component<components::Pose>(this->entity);
  if (!poseComp)
  {
    gzerr << "[MmWaveSensorPlugin] Entity has no Pose component.\n";
    return;
  }
  math::Pose3d sensorPose = poseComp->Data();

  // Create point cloud message
  msgs::PointCloudPacked pointCloudMsg;
  
  // Set header with timestamp
  pointCloudMsg.mutable_header()->mutable_stamp()->CopyFrom(convert<msgs::Time>(_info.simTime));
  
  // Add frame ID (sensor link name)
  auto nameComp = _ecm.Component<components::Name>(this->entity);
  if (nameComp)
  {
    pointCloudMsg.mutable_header()->add_data()->set_key("frame_id");
    pointCloudMsg.mutable_header()->mutable_data(0)->add_value(nameComp->Data());
  }

  // Define fields for the point cloud
  // x, y, z coordinates
  auto xField = pointCloudMsg.add_field();
  xField->set_name("x");
  xField->set_offset(0);
  xField->set_datatype(msgs::PointCloudPacked::Field::FLOAT32);
  xField->set_count(1);

  auto yField = pointCloudMsg.add_field();
  yField->set_name("y");
  yField->set_offset(4); // 4 bytes after x
  yField->set_datatype(msgs::PointCloudPacked::Field::FLOAT32);
  yField->set_count(1);

  auto zField = pointCloudMsg.add_field();
  zField->set_name("z");
  zField->set_offset(8); // 4 bytes after y
  zField->set_datatype(msgs::PointCloudPacked::Field::FLOAT32);
  zField->set_count(1);

  // Add velocity field (for Doppler)
  auto velField = pointCloudMsg.add_field();
  velField->set_name("velocity");
  velField->set_offset(12); // 4 bytes after z
  velField->set_datatype(msgs::PointCloudPacked::Field::FLOAT32);
  velField->set_count(1);

  // Add RCS field
  auto rcsField = pointCloudMsg.add_field();
  rcsField->set_name("rcs");
  rcsField->set_offset(16); // 4 bytes after velocity
  rcsField->set_datatype(msgs::PointCloudPacked::Field::FLOAT32);
  rcsField->set_count(1);

  // Add intensity field (signal strength)
  auto intensityField = pointCloudMsg.add_field();
  intensityField->set_name("intensity");
  intensityField->set_offset(20); // 4 bytes after rcs
  intensityField->set_datatype(msgs::PointCloudPacked::Field::FLOAT32);
  intensityField->set_count(1);

  // Set point step (total bytes per point)
  const int pointStep = 24; // 6 fields * 4 bytes
  pointCloudMsg.set_point_step(pointStep);
  pointCloudMsg.set_is_dense(true);
  
  // Use ray casting to populate the point cloud
  this->CastRays(sensorPose, _ecm, pointCloudMsg);
  
  // If no points were detected through ray casting and we're in debug mode,
  // add a single test point to verify the pipeline is working
  if (pointCloudMsg.width() == 0 && this->visualize)
  {
    gzmsg << "No points detected by ray casting, adding test point\n";
    
    // Add a single test point directly in front of the sensor
    pointCloudMsg.set_width(1);
    pointCloudMsg.set_height(1);
    pointCloudMsg.set_row_step(pointStep);
    
    std::vector<float> points;
    points.push_back(5.0f);  // x
    points.push_back(0.0f);  // y
    points.push_back(0.0f);  // z
    points.push_back(0.0f);  // velocity
    points.push_back(1.0f);  // rcs
    points.push_back(0.8f);  // intensity
    
    std::string data;
    data.resize(points.size() * sizeof(float));
    memcpy(&data[0], points.data(), points.size() * sizeof(float));
    pointCloudMsg.set_data(data);
  }

  if (this->pointCloudPublisher.HasConnections())
  {
    this->pointCloudPublisher.Publish(pointCloudMsg);
  }
}

// Register this plugin with Gazebo
GZ_ADD_PLUGIN(olympus_sim::MmWaveSensorPlugin,
              gz::sim::System,
              olympus_sim::MmWaveSensorPlugin::ISystemConfigure,
              olympus_sim::MmWaveSensorPlugin::ISystemPostUpdate)
