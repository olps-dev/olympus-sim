#include "MmWaveSensorPlugin.hh"

#include <filesystem>

#include <chrono>
#include <gz/plugin/Register.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/rendering/RenderEngine.hh> // For sceneFromFirstRenderEngine and engine
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
#include <cstdlib> // For setenv
#include "MmWaveSensorWslCompat.hh" // For WSL compatibility functions

using namespace olympus_sim;
using namespace gz;
using namespace gz::sim;

MmWaveSensorPlugin::MmWaveSensorPlugin() : 
    pointCloudLoader(std::make_unique<MmWavePointCloudLoader>())
{
    // Initialize time points separately to avoid constructor issues
    this->lastUpdateTime = std::chrono::steady_clock::duration(std::chrono::seconds(0));
    this->lastWorldModelUpdate = std::chrono::steady_clock::now();
  // Use a random seed for noise generation
  std::random_device rd;
  this->gaussianNoise = gz::math::NormalRealDist(rd());
}

void MmWaveSensorPlugin::Configure(const Entity &_entity,
                                   const std::shared_ptr<const sdf::Element> &_sdf,
                                   EntityComponentManager &_ecm,
                                   EventManager & /*_eventMgr*/)
{
  gzmsg << "[MmWaveSensorPlugin::Configure] Starting plugin configuration." << std::endl;
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
    // Do not return yet, try to initialize rendering components
  }

  // Initialize the ECS and events
  this->entity = _entity;

  // Initialize WSL compatibility mode
  char* wslEnv = std::getenv("WSL_DISTRO_NAME");
  bool isWSL = (wslEnv != nullptr);
  
  if (isWSL) 
  {
    gzmsg << "[MmWaveSensorPlugin] WSL environment detected. "
          << "Software rendering will be used." << std::endl;
          
    // Force software rendering in WSL environment
    setenv("LIBGL_ALWAYS_SOFTWARE", "1", 1);
  }
  
  // Initialize the point cloud loader
  if (!InitializePointCloudLoader())
  {
    gzerr << "[MmWaveSensorPlugin] Failed to initialize point cloud loader." << std::endl;
  }
  else
  {
    gzmsg << "[MmWaveSensorPlugin] Point cloud loader initialized successfully." << std::endl;
  }
}

bool MmWaveSensorPlugin::InitializePointCloudLoader()
{
  // Default point cloud directory is next to the plugin in sim/gazebo/pointclouds
  std::string pointCloudDir = "~/olympus-sim/sim/gazebo/pointclouds";
  
  // Try to find the absolute path
  char* homeDir = std::getenv("HOME");
  if (homeDir != nullptr)
  {
    pointCloudDir = std::string(homeDir) + "/olympus-sim/sim/gazebo/pointclouds";
  }
  
  // Expand the path if it has ~ (home directory)
  if (pointCloudDir.find('~') == 0)
  {
    if (homeDir != nullptr)
    {
      pointCloudDir.replace(0, 1, homeDir);
    }
  }
  
  // Create the directory if it doesn't exist
  try
  {
    if (!std::filesystem::exists(pointCloudDir))
    {
      gzmsg << "[MmWaveSensorPlugin] Creating point cloud directory: " << pointCloudDir << std::endl;
      std::filesystem::create_directories(pointCloudDir);
    }
  }
  catch (const std::exception& e)
  {
    gzerr << "[MmWaveSensorPlugin] Error creating point cloud directory: " << e.what() << std::endl;
  }
  
  // Initialize the point cloud loader
  bool success = false;
  try
  {
    success = this->pointCloudLoader->Initialize(pointCloudDir);
  }
  catch (const std::exception& e)
  {
    gzerr << "[MmWaveSensorPlugin] Error initializing point cloud loader: " << e.what() << std::endl;
    return false;
  }
  
  return success;
}

void MmWaveSensorPlugin::CastRays(const gz::math::Pose3d &_sensorPose,
                    const gz::sim::EntityComponentManager &_ecm, 
                    gz::msgs::PointCloudPacked &_pointCloudMsg)
{
  gzmsg << "[MmWaveSensorPlugin::CastRays] Entered CastRays. usePointCloudsOnly: " << this->usePointCloudsOnly << std::endl;
  // Debug message about sensor position
  gzdbg << "[MmWaveSensorPlugin] Sensor pose: pos=" << _sensorPose.Pos() 
        << ", rot=" << _sensorPose.Rot().Euler() << std::endl;

  if (!this->usePointCloudsOnly)
  {
    // ... (rest of the CastRays method remains the same)
  }
  else
  {
    // We rely on pre-computed point clouds, no scene needed
    if (this->pointCloudLoader)
    {
      gzmsg << "[MmWaveSensorPlugin] Using pre-computed point clouds for simulation" << std::endl;
      
      // 1. Get scene content information for selecting appropriate point cloud
      std::map<std::string, int> sceneObjectTypes;
      
      // Update our knowledge of scene contents periodically
      auto now = std::chrono::steady_clock::now();
      auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
          now - this->lastWorldModelUpdate);
      
      if (elapsed >= this->worldModelUpdatePeriod) 
      {
        this->lastWorldModelUpdate = now;
        gzmsg << "[MmWaveSensorPlugin::CastRays] Calling wsl_compat::UpdateSimplifiedWorldModel." << std::endl;
        // Use the WSL compatibility code to analyze world contents
        wsl_compat::UpdateSimplifiedWorldModel(
                                            this->entity,
                                            _ecm, 
                                            this->simplifiedWorldModel,
                                            this->wslWarningShown);
                                           
        // Extract object types for scene analysis
        for (const auto& [entityId, poseVel] : this->simplifiedWorldModel)
        {
          auto nameComp = _ecm.Component<gz::sim::components::Name>(entityId);
          if (nameComp)
          {
            const auto& name = nameComp->Data();
            sceneObjectTypes[name]++;
          }
        }
      }
      
      // 2. Get the appropriate point cloud based on scene contents
      auto pointCloud = this->pointCloudLoader->GetClosestMatchingPointCloud(_sensorPose, sceneObjectTypes);
      
      if (pointCloud)
      {
        // 3. Convert the point cloud to the point cloud message format
        this->pointCloudLoader->ConvertToPointCloudMsg(*pointCloud, _sensorPose, _pointCloudMsg);
        
        gzmsg << "[MmWaveSensorPlugin] Generated point cloud with " 
              << _pointCloudMsg.width() << " points from scenario: " 
              << pointCloud->name << std::endl;
      }
      else
      {
        gzerr << "[MmWaveSensorPlugin] Failed to get a point cloud from loader" << std::endl;
        // Set empty point cloud
        _pointCloudMsg.set_width(0);
        _pointCloudMsg.set_height(0);
        _pointCloudMsg.set_data("");
      }
    }
    else
    {
      gzerr << "[MmWaveSensorPlugin] Point cloud loader not initialized!" << std::endl;
      // Set empty point cloud
      _pointCloudMsg.set_width(0);
      _pointCloudMsg.set_height(0);
      _pointCloudMsg.set_data("");
    }
  }
}

void MmWaveSensorPlugin::PostUpdate(const gz::sim::UpdateInfo &_info,
                                     const gz::sim::EntityComponentManager &_ecm)
{
    gzmsg << "[MmWaveSensorPlugin::PostUpdate] Entered PostUpdate." << std::endl;
    // Skip if the simulation is paused
    if (_info.paused)
        return;

    // Rate limit the sensor updates
    auto currentTime = std::chrono::steady_clock::duration(_info.simTime);
    if ((currentTime - this->lastUpdateTime) < this->updatePeriod)
        return;
    this->lastUpdateTime = currentTime;

    // Try to acquire rendering scene if we don't have one yet and not in WSL compatibility mode
    if (!this->wslCompatMode && (!this->scene || !this->rayQuery)) {
        // Limit the number of attempts to avoid infinite retries
        int retryCount = ++this->sceneAcquisitionAttempts;
        if (retryCount > this->maxSceneAcquisitionAttempts) {
            // If we've exceeded max attempts, switch to WSL compatibility mode
            if (!this->wslCompatMode) {
                gzwarn << "[MmWaveSensorPlugin] Failed to acquire rendering scene after " 
                       << retryCount << " attempts. Switching to WSL compatibility mode." << std::endl;
                this->wslCompatMode = true;
            }
        }
        
        auto newScene = this->Scene();
        
        // Only update scene if we got a valid one
        if (newScene)
        {
            // Success! We found a scene
            gzmsg << "[MmWaveSensorPlugin::PostUpdate] Successfully acquired scene '" 
                  << newScene->Name() << "' on attempt " << retryCount << std::endl;
                  
            // Log detailed information about the scene to help with debugging
            gzmsg << "[MmWaveSensorPlugin::PostUpdate] Scene info - Name: '" << newScene->Name()
                  << "', Engine: '" << newScene->Engine()->Name() << "'" << std::endl;
            
            // Always update to the latest scene
            this->scene = newScene;
            
            // Reset rayQuery whenever scene changes (or if it doesn't exist yet)
            if (!this->rayQuery || this->rayQuery->Scene() != this->scene) {
                gzdbg << "[MmWaveSensorPlugin::PostUpdate] Scene changed or new, creating fresh RayQuery" << std::endl;
                this->rayQuery.reset();
                
                // Create new ray query for the scene with better error handling
                try {
                    this->rayQuery = this->scene->CreateRayQuery();
                    if (this->rayQuery) {
                        gzdbg << "[MmWaveSensorPlugin::PostUpdate] Successfully created RayQuery" << std::endl;
                    } else {
                        gzerr << "[MmWaveSensorPlugin::PostUpdate] Failed to create RayQuery - null pointer returned" << std::endl;
                    }
                } catch (const std::exception& e) {
                    gzerr << "[MmWaveSensorPlugin::PostUpdate] Exception creating RayQuery: " << e.what() << std::endl;
                }
            }
        } 
        else if (retryCount % 10 == 0 || retryCount == this->maxSceneAcquisitionAttempts) {
            // Log periodically to avoid spamming, but ensure last attempt is always logged
            gzwarn << "[MmWaveSensorPlugin::PostUpdate] Still trying to find rendering scene "
                   << "(attempt " << retryCount << "" 
                   << (retryCount == this->maxSceneAcquisitionAttempts ? ", FINAL ATTEMPT" : "") 
                   << "). This is normal in some environments." << std::endl;
        }
    }
    
    gzmsg << "[MmWaveSensorPlugin::PostUpdate] Checking WSL compatibility mode. wslCompatMode: " << this->wslCompatMode << std::endl;
    // If we're in WSL compatibility mode, no need to check for scene or rayQuery
    // Otherwise, if scene or rayQuery is still not available, log warning
    if (!this->wslCompatMode && (!this->scene || !this->rayQuery))
    {
        static int error_count = 0;
        if (error_count % 100 == 0) { // Log periodically
             gzwarn << "[MmWaveSensorPlugin::PostUpdate] Scene or RayQuery unavailable. "
                    << "Will keep trying." << std::endl;
        }
        error_count++;
        return;
    }

  // Only skip ray-casting if scene or rayQuery aren't available AND we're not in WSL compatibility mode
  // If we're in WSL compatibility mode, we should continue even without scene or rayQuery
  if ((!this->scene || !this->rayQuery) && !this->wslCompatMode) {
    // Print warning message once per second to avoid console flooding
    static auto lastWarningTime = std::chrono::steady_clock::now();
    auto currentTime = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(currentTime - lastWarningTime).count();
    
    if (elapsed >= 5) { // Only warn every 5 seconds
      gzdbg << "[MmWaveSensorPlugin] Skipping normal update - ray casting not available.\n";
      lastWarningTime = currentTime;
    }
    return;
  }

  // Get sensor pose
  auto poseComp = _ecm.Component<components::Pose>(this->entity);
  if (!poseComp)
  {
    gzerr << "[MmWaveSensorPlugin] Entity has no Pose component.\n";
    return;
  }
  math::Pose3 sensorPose = poseComp->Data();
  
  // Get the world pose of the sensor
  math::Pose3 worldSensorPose = gz::sim::worldPose(this->entity, _ecm);
  
  // Update the simplified world model for WSL compatibility mode if needed
  if (this->wslCompatMode) {
    auto currentTime = std::chrono::steady_clock::now();
    if ((currentTime - this->lastWorldModelUpdate) > this->worldModelUpdatePeriod) {
      olympus_sim::wsl_compat::UpdateSimplifiedWorldModel(this->entity, _ecm, this->simplifiedWorldModel, this->wslWarningShown);
      this->lastWorldModelUpdate = currentTime;
      gzmsg << "[MmWaveSensorPlugin] Updated simplified world model with " 
            << this->simplifiedWorldModel.size() << " entities." << std::endl;
    }
  }

  // Debug - Entity hierarchy and pose information (reduced in WSL mode)
  if (!this->wslCompatMode) {
    gzmsg << "[MmWaveSensorPlugin] Current entity ID: " << this->entity << "\n";
  }

  // Get entity name for debug
  auto entityNameComp = _ecm.Component<components::Name>(this->entity);
  if (entityNameComp)
  {
    gzmsg << "[MmWaveSensorPlugin] Entity name: " << entityNameComp->Data() << "\n";
  }

  // Get parent entity and its pose
  auto parentEntity = _ecm.ParentEntity(this->entity);
  if (parentEntity != gz::sim::kNullEntity)
  {
    gzmsg << "[MmWaveSensorPlugin] Parent entity ID: " << parentEntity << "\n";
    
    // Get parent name
    auto parentNameComp = _ecm.Component<components::Name>(parentEntity);
    if (parentNameComp)
    {
      gzmsg << "[MmWaveSensorPlugin] Parent entity name: " << parentNameComp->Data() << "\n";
    }
    
    // Get parent pose
    auto parentPoseComp = _ecm.Component<components::Pose>(parentEntity);
    if (parentPoseComp)
    {
      math::Pose3 parentPose = parentPoseComp->Data();
      gzmsg << "[MmWaveSensorPlugin] Parent entity pose: " << parentPose << "\n";
    }
    
    // Try grandparent too (model entity)
    auto grandparentEntity = _ecm.ParentEntity(parentEntity);
    if (grandparentEntity != gz::sim::kNullEntity)
    {
      gzmsg << "[MmWaveSensorPlugin] Grandparent entity ID: " << grandparentEntity << "\n";
      
      // Get grandparent name
      auto grandparentNameComp = _ecm.Component<components::Name>(grandparentEntity);
      if (grandparentNameComp)
      {
        gzmsg << "[MmWaveSensorPlugin] Grandparent entity name: " << grandparentNameComp->Data() << "\n";
      }
      
      // Get grandparent pose
      auto grandparentPoseComp = _ecm.Component<components::Pose>(grandparentEntity);
      if (grandparentPoseComp)
      {
        math::Pose3 grandparentPose = grandparentPoseComp->Data();
        gzmsg << "[MmWaveSensorPlugin] Grandparent entity pose: " << grandparentPose << "\n";
      }
    }
  }
  
  // Instead of just using the entity's pose, try to get the world pose
  // Already have worldSensorPose from above
  gzmsg << "[MmWaveSensorPlugin] Entity local pose: " << sensorPose << "\n";
  gzmsg << "[MmWaveSensorPlugin] Entity world pose: " << worldSensorPose << "\n";

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
  
  try {
    // Use pre-computed point clouds if enabled (primary approach)
    if (this->usePointCloudsOnly) {
      gzmsg << "[MmWaveSensorPlugin] Using pre-computed point cloud simulation approach" << std::endl;
      
      // Extract scene content information for better point cloud selection
      std::map<std::string, int> sceneObjectTypes;
      
      // Update our knowledge of scene contents periodically
      auto now = std::chrono::steady_clock::now();
      auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
          now - this->lastWorldModelUpdate);
      
      if (elapsed >= this->worldModelUpdatePeriod) {
        this->lastWorldModelUpdate = now;
        // Use the WSL compatibility code to analyze world contents
        olympus_sim::wsl_compat::UpdateSimplifiedWorldModel(
          this->entity,
          _ecm, 
          this->simplifiedWorldModel, 
          this->wslWarningShown);
        
        // Extract object types for scene analysis
        for (const auto& [entityId, poseVel] : this->simplifiedWorldModel) {
          auto nameComp = _ecm.Component<gz::sim::components::Name>(entityId);
          if (nameComp) {
            const auto& name = nameComp->Data();
            sceneObjectTypes[name]++;
          }
        }
      }
      
      // Get the appropriate point cloud based on scene contents
      try {
        auto pointCloud = this->pointCloudLoader->GetClosestMatchingPointCloud(worldSensorPose, sceneObjectTypes);
        
        if (pointCloud) {
          // Convert the point cloud to the point cloud message format
          this->pointCloudLoader->ConvertToPointCloudMsg(*pointCloud, worldSensorPose, pointCloudMsg);
          
          // Show point count for debugging
          gzmsg << "[MmWaveSensorPlugin] Generated " << pointCloudMsg.width() 
                << " points from pre-computed point cloud scenario: " 
                << pointCloud->name << std::endl;
        }
        else {
          gzwarn << "[MmWaveSensorPlugin] Failed to get a point cloud, using empty point cloud" << std::endl;
          // Set empty point cloud
          pointCloudMsg.set_width(0);
          pointCloudMsg.set_height(0);
          pointCloudMsg.set_data("");
        }
      }
      catch (const std::exception& e) {
        gzerr << "[MmWaveSensorPlugin] ERROR in point cloud processing: " << e.what() << std::endl;
        throw; // Rethrow to outer catch block
      }
    }
    // In WSL compatibility mode, use the simplified approach as fallback
    else if (this->wslCompatMode) {
      gzmsg << "[MmWaveSensorPlugin] ** USING WSL COMPATIBILITY MODE **" << std::endl;
      
      // Update the simplified world model first
      gzmsg << "[MmWaveSensorPlugin] Updating simplified world model..." << std::endl;
      olympus_sim::wsl_compat::UpdateSimplifiedWorldModel(
        this->entity, 
        _ecm, 
        this->simplifiedWorldModel, 
        this->wslWarningShown);
      
      // Generate the simulated data
      gzmsg << "[MmWaveSensorPlugin] Generating simulated data in WSL compatibility mode..." << std::endl;
      try {
        olympus_sim::wsl_compat::GenerateSimulatedData(
          this->entity,
          worldSensorPose, 
          _ecm, 
          pointCloudMsg, 
          this->simplifiedWorldModel,
          this->horizontalFov,
          this->horizontalResolution,
          this->verticalFov,
          this->verticalResolution,
          this->minRange,
          this->maxRange,
          this->noiseMean,
          this->noiseStdDev,
          this->defaultRCS,
          this->minRCS,
          this->maxRadialVelocity
        );
        
        // Show point count for debugging
        size_t pointCount = pointCloudMsg.data().size() / 
                          (pointCloudMsg.point_step() > 0 ? pointCloudMsg.point_step() : 1);
        gzmsg << "[MmWaveSensorPlugin] --> Successfully generated " << pointCount 
              << " points in WSL compatibility mode" << std::endl;
      } 
      catch (const std::exception& e) {
        gzerr << "[MmWaveSensorPlugin] ERROR in GenerateSimulatedData: " << e.what() << std::endl;
        throw; // Rethrow to outer catch block
      }
    } else {
      // Last resort: fall back to ray casting for sensor data generation
      gzmsg << "[MmWaveSensorPlugin] Using ray casting for sensing (not recommended)" << std::endl;
      this->CastRays(worldSensorPose, _ecm, pointCloudMsg);
    }
    
    // If no points were detected and we're in debug mode,
    // add a single test point to verify the pipeline is working
    if (pointCloudMsg.width() == 0 && this->visualize) {
      // Add a single test point at 5 meters in front of the sensor
      gzmsg << "[MmWaveSensorPlugin] No points detected, adding test point" << std::endl;
      
      // Clear any existing fields and add all required ones
      pointCloudMsg.clear_field();
      
      // Add x field
      auto field = pointCloudMsg.add_field();
      field->set_name("x");
      field->set_offset(0);
      field->set_datatype(msgs::PointCloudPacked::Field::FLOAT32);
      field->set_count(1);
      
      // Add y field
      field = pointCloudMsg.add_field();
      field->set_name("y");
      field->set_offset(4);
      field->set_datatype(msgs::PointCloudPacked::Field::FLOAT32);
      field->set_count(1);
      
      // Add z field
      field = pointCloudMsg.add_field();
      field->set_name("z");
      field->set_offset(8);
      field->set_datatype(msgs::PointCloudPacked::Field::FLOAT32);
      field->set_count(1);
      
      // Point at 5 meters in front of the sensor
      float x = 5.0f;
      float y = 0.0f;
      float z = 0.0f;
      
      pointCloudMsg.set_point_step(12);  // 3 fields * 4 bytes (float32)
      pointCloudMsg.set_height(1);
      pointCloudMsg.set_width(1);
      pointCloudMsg.set_row_step(12);
      pointCloudMsg.set_is_dense(true);
      
      pointCloudMsg.mutable_data()->resize(12);
      std::memcpy(&pointCloudMsg.mutable_data()->at(0), &x, 4);
      std::memcpy(&pointCloudMsg.mutable_data()->at(4), &y, 4);
      std::memcpy(&pointCloudMsg.mutable_data()->at(8), &z, 4);
      
      gzmsg << "[MmWaveSensorPlugin] Added test point at (" 
            << x << ", " << y << ", " << z << ")" << std::endl;
    }
    
    // Publish point cloud message
    if (!this->pointCloudPublisher.Publish(pointCloudMsg)) {
      gzwarn << "[MmWaveSensorPlugin] Failed to publish point cloud message." << std::endl;
    } else {
      size_t pointCount = pointCloudMsg.data().size() / 
                        (pointCloudMsg.point_step() > 0 ? pointCloudMsg.point_step() : 1);
      gzmsg << "[MmWaveSensorPlugin] Successfully published " << pointCount
            << " points to topic: " << this->topicName 
            << (this->wslCompatMode ? " (WSL compatibility mode)" : "") << std::endl;
    }
  } // End of try block
  catch (const std::exception& e) {
    gzerr << "[MmWaveSensorPlugin] Exception during sensor data generation: " << e.what() << std::endl;
  }
  catch (...) {
    gzerr << "[MmWaveSensorPlugin] Unknown exception during sensor data generation" << std::endl;
  }

} // End of PostUpdate method

// GenerateSimulatedData implementation moved to MmWaveSensorWslCompat.cc

gz::rendering::ScenePtr MmWaveSensorPlugin::Scene() const
{
  // WSL environments often have issues with rendering scenes
  // Add detailed logging to help diagnose the problem
  gzdbg << "[MmWaveSensorPlugin] Attempting to get rendering scene...\n";
  
  // Check if we're running in WSL - this affects how aggressively we search
  static bool checkedWSL = false;
  static bool isWSL = false;
  if (!checkedWSL) {
    char* wslEnv = std::getenv("WSL_DISTRO_NAME");
    isWSL = (wslEnv != nullptr);
    checkedWSL = true;
    
    if (isWSL) {
      gzdbg << "[MmWaveSensorPlugin::Scene] WSL detected, will avoid scene creation to prevent crashes" << std::endl;
    }
  }
  
  // In WSL environments, avoid any scene creation operations that cause segmentation faults
  // Return nullptr immediately to force WSL compatibility mode
  if (isWSL) {
    gzdbg << "[MmWaveSensorPlugin::Scene] WSL detected - returning nullptr to force compatibility mode" << std::endl;
    return nullptr;
  }
  
  // ====== APPROACH 1: Try the scene belonging to the current world ======
  // In Gazebo Sim, the world name should match the main scene name
  // This gives us the best chance of getting the scene with simulation objects
  
  // Note: Since this is a const method, we're using static variables to maintain state
  static std::string lastWorldName = "";
  static std::string currentWorldName = "mmwave_test"; // Default world name, will be updated if available
  
  // Try to update current world name if not already set
  if (lastWorldName != currentWorldName)
  {
    gzmsg << "[MmWaveSensorPlugin] Looking for rendering scene for world: '" << currentWorldName << "'\n";
    lastWorldName = currentWorldName;
  }
  
  // Try all loaded engines to find the scene with this world name
  const std::vector<std::string> engineNames = {"ogre2", "ogre"};
  
  // Skip scene creation in WSL as it causes segmentation faults
  // We'll rely on existing scenes or fall back to WSL compatibility mode
  
  // Continue with original approaches if the new approach didn't work
  for (const auto& engineName : engineNames)
  {
    try
    {
      auto loadedEngine = gz::rendering::engine(engineName);
      if (!loadedEngine) continue;
      
      // Try to get scene with world name
      auto scene = loadedEngine->SceneByName(currentWorldName);
      if (scene)
      {
        gzmsg << "[MmWaveSensorPlugin] Found scene '" << scene->Name() 
              << "' matching world name in engine '" << engineName << "'\n";
        return scene;
      }
    }
    catch (...) {} // Silent catch - just try next engine
  }
  
  // ====== APPROACH 2: Get from first render engine ======
  // This is more likely to be the GUI's scene which contains simulation objects
  auto scene = gz::rendering::sceneFromFirstRenderEngine();
  if (scene)
  {
    gzmsg << "[MmWaveSensorPlugin] Successfully got scene '" << scene->Name() 
          << "' from first render engine '" << scene->Engine()->Name() << "'\n";
    return scene;
  }
  
  gzdbg << "[MmWaveSensorPlugin] Failed to get scene from first render engine\n";

  // ====== APPROACH 3: Try with explicit engine names and common scene names ======
  // In Gazebo Sim, the GUI typically creates scenes with predictable names
  const std::vector<std::string> commonSceneNames = {
    "scene", // Most common default name
    "default",
    "main",
    "world",
    "mmwave_test" // Try again with world name
  };
  
  gzdbg << "[MmWaveSensorPlugin] Trying explicit engines and scene names...\n";
  
  for (const auto& engineName : engineNames)
  {
    try
    {
      auto loadedEngine = gz::rendering::engine(engineName);
      if (!loadedEngine) continue;
      
      gzdbg << "[MmWaveSensorPlugin] Checking engine '" << engineName << "' for scenes\n";
      
      // Look through common scene names
      for (const auto& sceneName : commonSceneNames)
      {
        scene = loadedEngine->SceneByName(sceneName);
        if (scene)
        {
          gzmsg << "[MmWaveSensorPlugin] Found scene '" << sceneName 
                << "' in engine '" << engineName << "'\n";
          return scene;
        }
      }
      
      // If really necessary, inspect any scene in this engine
      // In Gazebo, the engine typically only has one scene
      try {
        // Get first scene by iterating through all scene IDs
        // This is implementation-specific and may not work in all Gazebo versions
        for (uint32_t i = 0; i < 100; i++) { // Try first 100 potential scene IDs
          scene = loadedEngine->SceneById(i);
          if (scene) {
            gzmsg << "[MmWaveSensorPlugin] Found scene by ID " << i 
                  << ", name: '" << scene->Name() << "' in engine '" << engineName << "'\n";
            return scene;
          }
        }
      } catch (...) {
        // Silently continue if this approach fails
      }
    }
    catch (...) {
      // Silent catch - just try next engine
    }
  }
  
  // Skip scene creation entirely in WSL as it causes segmentation faults
  // We'll fall back to WSL compatibility mode instead
  
  // Do NOT create a new scene as it would be empty and causes crashes in WSL
  gzwarn << "[MmWaveSensorPlugin] Failed to find an existing scene.\n"
         << "Ray casting will not work until a scene is available.\n"
         << "The plugin will keep trying in PostUpdate().\n";
  
  // In WSL/WSLg, it often takes time for the rendering scene to be available
  // Rather than creating an empty scene, return nullptr and let PostUpdate() retry
  return nullptr;
}

// Register this plugin with Gazebo
GZ_ADD_PLUGIN(olympus_sim::MmWaveSensorPlugin,
              gz::sim::System,
              olympus_sim::MmWaveSensorPlugin::ISystemConfigure,
              olympus_sim::MmWaveSensorPlugin::ISystemPostUpdate)
