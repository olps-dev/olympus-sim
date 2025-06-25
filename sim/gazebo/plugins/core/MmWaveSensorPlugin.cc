#include "core/MmWaveSensorPlugin.hh"

#include <filesystem>
#include <chrono>
#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <cstring>
#include <cmath>
#include <cstdlib> // For setenv

#include <gz/plugin/Register.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/common/Console.hh>

#include "messaging/MmWaveMessageHandler.hh"
#include "core/MmWaveSensorWslCompat.hh"
#include "raycast/MmWaveSensorRay.hh"

using namespace olympus_sim;
using namespace gz;
using namespace gz::sim;

MmWaveSensorPlugin::MmWaveSensorPlugin()
{
  // Initialize time points separately to avoid constructor issues
  this->lastUpdateTime = std::chrono::steady_clock::duration(std::chrono::seconds(0));
  this->lastWorldModelUpdate = std::chrono::steady_clock::now();
  
  // Use a random seed for noise generation
  std::random_device rd;
  this->randomEngine = std::mt19937(rd());
  this->gaussianNoise = std::normal_distribution<double>(0.0, 1.0);
  
  // Initialize component objects
  this->pointCloudLoader = std::make_unique<MmWavePointCloudLoader>();
  this->sceneManager = std::make_unique<MmWaveSensorScene>();
}

void MmWaveSensorPlugin::Configure(const Entity &_entity,
                                   const std::shared_ptr<const sdf::Element> &_sdf,
                                   EntityComponentManager &_ecm,
                                   EventManager & /*_eventMgr*/)
{
  gzmsg << "\n\n**************************************************\n";
  gzmsg << "*      LOADING MMWAVE PLUGIN (REFACTORED)       *\n";
  gzmsg << "**************************************************\n\n" << std::endl;
  gzmsg << "[MmWaveSensorPlugin::Configure] Starting plugin configuration." << std::endl;
  
  // Store the entity reference
  this->entity = _entity;

  // Get plugin name for logging
  std::string pluginName = "MmWaveSensorPlugin";
  if (_sdf->HasAttribute("name"))
  {
    pluginName = _sdf->Get<std::string>("name");
  }
  
  // Check that entity has a name component
  auto nameComp = _ecm.Component<components::Name>(this->entity);
  if (nameComp)
  {
    gzmsg << "[" << pluginName << "] Configuring plugin for entity [" 
          << nameComp->Data() << "]" << std::endl;
  }
  else
  {
    gzwarn << "[" << pluginName << "] Entity has no name component" << std::endl;
  }

  // Load configuration from SDF
  if (!this->config.Load(_sdf, pluginName))
  {
    gzerr << "[" << pluginName << "] Failed to load configuration" << std::endl;
    return;
  }

  // Initialize Gazebo transport publisher
  this->pointCloudPublisher = this->node.Advertise<msgs::PointCloudPacked>(this->config.topicName);
  if (!this->pointCloudPublisher)
  {
    gzerr << "[" << pluginName << "] Failed to create publisher on topic [" 
          << this->config.topicName << "]." << std::endl;
    // Do not return yet, try to initialize other components
  }

  // Check for WSL environment
  char* wslEnv = std::getenv("WSL_DISTRO_NAME");
  bool isWSL = (wslEnv != nullptr);
  
  if (isWSL) 
  {
    gzmsg << "[MmWaveSensorPlugin] WSL environment detected. "
          << "Software rendering will be used." << std::endl;
          
    // Force software rendering in WSL environment
    setenv("LIBGL_ALWAYS_SOFTWARE", "1", 1);
    this->wslCompatMode = true;
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
  {
    gzmsg << "[MmWaveSensorPlugin] Point cloud loader not needed for this mode." << std::endl;
    return true;
  }
  
  // Get the data directory from environment variable
  char* olympusHome = std::getenv("OLYMPUS_HOME");
  if (!olympusHome)
  {
    gzerr << "[MmWaveSensorPlugin] OLYMPUS_HOME environment variable not set. "
          << "Cannot load point clouds." << std::endl;
    return false;
  }
  
  // Construct path to point cloud data directory
  std::filesystem::path dataDir(olympusHome);
  dataDir /= "data";  
  dataDir /= "point_clouds";
  
  // Validate the directory exists
  if (!std::filesystem::exists(dataDir) || !std::filesystem::is_directory(dataDir))
  {
    gzerr << "[MmWaveSensorPlugin] Point cloud data directory not found at: "
          << dataDir << std::endl;
    return false;
  }
  
  // Load the point clouds
  bool loadSuccess = false;
  try
  {
    loadSuccess = this->pointCloudLoader->LoadPointCloudsFromDirectory(dataDir.string());
  }
  catch (const std::exception& e)
  {
    gzerr << "[MmWaveSensorPlugin] Exception while loading point clouds: "
          << e.what() << std::endl;
    return false;
  }
  
  if (!loadSuccess)
  {
    gzerr << "[MmWaveSensorPlugin] Failed to load point clouds from directory: "
          << dataDir << std::endl;
    return false;
  }
  
  gzmsg << "[MmWaveSensorPlugin] Successfully loaded point clouds from directory: "
        << dataDir << std::endl;
        
  return true;
}

// CastRays method removed - functionality is now in MmWaveSensorRay class

void MmWaveSensorPlugin::PostUpdate(const gz::sim::UpdateInfo &_info,
                                     const gz::sim::EntityComponentManager &_ecm)
{
    gzdbg << "[MmWaveSensorPlugin::PostUpdate] Entered PostUpdate." << std::endl;
    // Skip if the simulation is paused
    if (_info.paused)
        return;

    // Rate limit the sensor updates based on configured update rate
    auto currentTime = std::chrono::steady_clock::duration(_info.simTime);
    if ((currentTime - this->lastUpdateTime) < this->config.updatePeriod)
        return;
    this->lastUpdateTime = currentTime;

    // Try to acquire rendering scene if needed and not in WSL compatibility mode
    if (!this->wslCompatMode && !this->sceneManager->HasValidScene()) 
    {
        // Use the scene manager to attempt scene acquisition
        if (!this->sceneManager->AcquireScene())
        {
            // If we've exceeded max attempts, switch to WSL compatibility mode
            if (this->sceneManager->GetAcquisitionAttempts() > this->sceneManager->GetMaxAcquisitionAttempts())
            {
                gzwarn << "[MmWaveSensorPlugin] Failed to acquire rendering scene after " 
                       << this->sceneManager->GetAcquisitionAttempts() << " attempts. "
                       << "Switching to WSL compatibility mode." << std::endl;
                this->wslCompatMode = true;
            }
        }
        else
        {  
            gzmsg << "[MmWaveSensorPlugin::PostUpdate] Successfully acquired scene on attempt "
                  << this->sceneManager->GetAcquisitionAttempts() << std::endl;
        }
    }
    
    // If we're in WSL compatibility mode, we don't need rendering
    // Otherwise, ensure we have a valid scene with ray casting capability
    if (!this->wslCompatMode && !this->sceneManager->HasValidScene())
    {
        static int error_count = 0;
        if (error_count % 100 == 0) { // Log periodically
             gzwarn << "[MmWaveSensorPlugin::PostUpdate] Scene or RayQuery unavailable. "
                    << "Will keep trying." << std::endl;
        }
        error_count++;
        return;
    }
    
    // Only skip ray-casting if scene isn't available AND we're not in WSL compatibility mode
    // If we're in WSL compatibility mode, we should continue even without scene
    if (!this->sceneManager->HasValidScene() && !this->wslCompatMode) {
        // Print warning message once per second to avoid console flooding
        static auto lastWarningTime = std::chrono::steady_clock::now();
        auto currentTime = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(currentTime - lastWarningTime).count();
        
        if (elapsed >= 5) { // Only warn every 5 seconds
            gzdbg << "[MmWaveSensorPlugin] Skipping update - ray casting not available" << std::endl;
            lastWarningTime = currentTime;
        }
        return;
    }

    // Get sensor pose in world frame
    math::Pose3d worldSensorPose = gz::sim::worldPose(this->entity, _ecm);
    if (worldSensorPose == math::Pose3d::Zero) {
        gzerr << "[MmWaveSensorPlugin] Failed to get valid world pose for sensor" << std::endl;
        return;
    }
  
    // Update the simplified world model for WSL compatibility mode if needed
    if (this->wslCompatMode) {
        auto currentTime = std::chrono::steady_clock::now();
        if ((currentTime - this->lastWorldModelUpdate) > this->config.worldModelUpdatePeriod) {
            olympus_sim::wsl_compat::UpdateSimplifiedWorldModel(
                this->entity, _ecm, this->simplifiedWorldModel, this->wslWarningShown);
            this->lastWorldModelUpdate = currentTime;
            gzdbg << "[MmWaveSensorPlugin] Updated simplified world model with " 
                  << this->simplifiedWorldModel.size() << " entities" << std::endl;
        }
    }

    // Get sensor name for frame ID and debug information
    std::string sensorName = "mmwave_sensor";
    auto nameComp = _ecm.Component<components::Name>(this->entity);
    if (nameComp)
    {
        sensorName = nameComp->Data();
        gzdbg << "[MmWaveSensorPlugin] Sensor name: " << sensorName << std::endl;
    }
  
    // Create point cloud message using message handler
    msgs::PointCloudPacked pointCloudMsg = 
        this->messageHandler->CreatePointCloudMessage(_info.simTime.count(), sensorName);
  
    try {
        // Extract scene content information (needed for point cloud selection)
        std::map<std::string, int> sceneObjectTypes;
        for (const auto& [entityId, poseVel] : this->simplifiedWorldModel) {
            auto nameComp = _ecm.Component<components::Name>(entityId);
            if (nameComp) {
                sceneObjectTypes[nameComp->Data()]++;
            }
        }
        
        // Select appropriate sensing approach based on configuration
        if (this->config.usePointCloudsOnly) {
            // APPROACH 1: Use pre-computed point clouds from files
            gzdbg << "[MmWaveSensorPlugin] Using pre-computed point cloud approach" << std::endl;
            
            auto pointCloud = this->pointCloudLoader->GetClosestMatchingPointCloud(
                worldSensorPose, sceneObjectTypes);
            
            if (pointCloud) {
                this->pointCloudLoader->ConvertToPointCloudMsg(
                    *pointCloud, worldSensorPose, pointCloudMsg);
                gzdbg << "[MmWaveSensorPlugin] Generated " << pointCloudMsg.width() 
                      << " points from pre-computed scenario: " << pointCloud->name << std::endl;
            } 
            else {
                gzwarn << "[MmWaveSensorPlugin] No matching point cloud found" << std::endl;
            }
        }
        else if (this->wslCompatMode) {
            // APPROACH 2: Use WSL compatibility mode with simplified world model
            gzdbg << "[MmWaveSensorPlugin] Using WSL compatibility mode" << std::endl;
            
            olympus_sim::wsl_compat::GenerateSimulatedData(
                this->entity,
                worldSensorPose, 
                _ecm,
                pointCloudMsg,
                this->simplifiedWorldModel,
                this->config.horizontalFov,
                this->config.horizontalResolution,
                this->config.verticalFov,
                this->config.verticalResolution,
                this->config.minRange,
                this->config.maxRange,
                this->config.noiseMean,
                this->config.noiseStdDev,
                this->config.defaultRCS,
                this->config.minRCS,
                this->config.maxRadialVelocity);
        }
        else {
            // APPROACH 3: Use normal ray query mode with full Gazebo rendering
            gzdbg << "[MmWaveSensorPlugin] Using ray casting simulation" << std::endl;
            
            // Create ray sensor object if needed
            if (!this->raySensor) {
                this->raySensor = std::make_unique<MmWaveSensorRay>();
                this->raySensor->SetRayQuery(this->sceneManager->GetRayQuery());
            }
            
            // Cast rays using the ray sensor object
            this->raySensor->CastRays(
                worldSensorPose,
                _ecm,
                pointCloudMsg,
                this->config.horizontalFov,
                this->config.horizontalResolution,
                this->config.verticalFov,
                this->config.verticalResolution,
                this->config.minRange,
                this->config.maxRange,
                this->config.noiseMean,
                this->config.noiseStdDev,
                this->config.defaultRCS,
                this->config.minRCS,
                this->config.maxRadialVelocity,
                this->gaussianNoise);
        }
        
        // Add test point if point cloud is empty and visualization is enabled
        if (pointCloudMsg.width() == 0 && this->config.visualize) {
            gzdbg << "[MmWaveSensorPlugin] No points detected, adding test point" << std::endl;
            this->messageHandler->AddTestPoint(pointCloudMsg);
        }
        
        // Publish point cloud data
        this->pointCloudPublisher.Publish(pointCloudMsg);
    }
    catch (const std::exception& e) {
        gzerr << "[MmWaveSensorPlugin] Exception during sensor data generation: " 
              << e.what() << std::endl;
    }
    catch (...) {
        gzerr << "[MmWaveSensorPlugin] Unknown exception during sensor data generation" << std::endl;
    }

    // End of PostUpdate method
}

// Scene() method removed - functionality is now in MmWaveSensorScene class
// CastRays() method removed - functionality is now in MmWaveSensorRay class

// Register this plugin with Gazebo
GZ_ADD_PLUGIN(olympus_sim::MmWaveSensorPlugin,
              gz::sim::System,
              olympus_sim::MmWaveSensorPlugin::ISystemConfigure,
              olympus_sim::MmWaveSensorPlugin::ISystemPostUpdate)
