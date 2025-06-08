#include "MmWaveSensorPlugin.hh"

#include <gz/plugin/Register.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Sensor.hh> // May need for sensor metadata
#include <gz/math/Rand.hh> // For noise simulation

// For ray casting (will need to include appropriate gz-rendering or gz-physics headers)
// #include <gz/rendering/Scene.hh>
// #include <gz/rendering/RayQuery.hh>


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
  // Add loading for other parameters (noise, etc.)

  gzmsg << "[" << pluginName << "] Parameters loaded: \n"
        << "  Topic: " << this->topicName << "\n"
        << "  Update Rate: " << this->updateRate << " Hz\n"
        << "  Horizontal FOV: " << this->horizontalFov << " rad\n"
        << "  Vertical FOV: " << this->verticalFov << " rad\n"
        << "  Min Range: " << this->minRange << " m\n"
        << "  Max Range: " << this->maxRange << " m\n"
        << "  Horizontal Resolution: " << this->horizontalResolution << "\n"
        << "  Vertical Resolution: " << this->verticalResolution << "\n";


  // Initialize Gazebo transport publisher
  this->pointCloudPublisher = this->node.Advertise<msgs::PointCloudPacked>(this->topicName);
  if (!this->pointCloudPublisher)
  {
    gzerr << "[" << pluginName << "] Failed to create publisher on topic [" << this->topicName << "].\n";
  }
}

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

  // --- Core mmWave Simulation Logic will go here ---
  // 1. Get rendering scene (needed for ray casting)
  //    - This part needs careful implementation, might involve accessing a global scene manager
  //    - auto scene = rendering::sceneFromFirstRenderEngine(); (or similar)

  // 2. Perform Ray Casting:
  //    - Create a rendering::RayQuery object for each ray.
  //    - Set ray origin (sensorPose.Pos()) and direction (based on FOV and resolution).
  //    - scene->CreateRayQuery(rayQuery);
  //    - For each ray:
  //      - Get intersection point: rayQuery->ClosestPoint();
  //      - Get intersected entity: rayQuery->ClosestPointFidelity().Entity();

  // 3. Process Detections:
  //    - If intersection is valid and within range:
  //      - Get entity's velocity (from _ecm, e.g., components::LinearVelocity)
  //      - Calculate relative radial velocity.
  //      - Estimate RCS (requires models to have RCS properties or a default).
  //      - Simulate SNR, power, noise.

  // 4. Populate PointCloudPacked message:
  //    - msgs::PointCloudPacked pointCloudMsg;
  //    - pointCloudMsg.mutable_header()->mutable_stamp()->CopyFrom(convert<msgs::Time>(_info.simTime));
  //    - pointCloudMsg.set_height(1); // For an unstructured point cloud
  //    - pointCloudMsg.set_width(num_detected_points);
  //    - Add fields: "x", "y", "z", "radial_velocity", "rcs", "intensity" (or power/snr)
  //      - pointCloudMsg.add_field()->set_name("x"); ...
  //    - For each detected point, add its data to pointCloudMsg.mutable_data()

  // --- Placeholder: Publish an empty point cloud for now ---
  msgs::PointCloudPacked pointCloudMsg;
  // Set header
  pointCloudMsg.mutable_header()->mutable_stamp()->CopyFrom(convert<msgs::Time>(_info.simTime));
  // TODO: Set frame_id if needed, usually the sensor link name
  // pointCloudMsg.mutable_header()->mutable_frame_id()->set_data("mmwave_sensor_link");

  // Define fields (example)
  // We need x, y, z, radial_velocity, rcs, intensity/snr
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

  // Add more fields for radial_velocity, rcs, etc.
  // The point_step will be the sum of bytes of all fields for one point.
  // pointCloudMsg.set_point_step(12); // For x, y, z only (3 * 4 bytes)
  // pointCloudMsg.set_is_bigendian(false);
  // pointCloudMsg.set_is_dense(true); // No invalid points

  // For now, an empty cloud
  pointCloudMsg.set_width(0);
  pointCloudMsg.set_height(1);
  pointCloudMsg.set_point_step(12); // Placeholder, update when fields are added
  pointCloudMsg.set_is_dense(true);


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
