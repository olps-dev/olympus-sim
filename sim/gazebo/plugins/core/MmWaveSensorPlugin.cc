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
  bool isWSL = (wslEnv != nullptr) || std::filesystem::exists("/proc/sys/fs/binfmt_misc/WSLInterop");
  
  // Set up environment variables for better OpenGL compatibility in WSL
  if (isWSL) {
    gzmsg << "[MmWaveSensorPlugin] WSL detected: Setting up WSL rendering environment" << std::endl;
    
    // These environment variables help with WSL OpenGL compatibility
    setenv("LIBGL_ALWAYS_SOFTWARE", "1", 1);  // Force software rendering
    setenv("MESA_GL_VERSION_OVERRIDE", "3.3", 1);  // Set OpenGL version
    setenv("LIBGL_ALWAYS_INDIRECT", "1", 1);  // Use indirect rendering
    setenv("OGRE_RTT_MODE", "Copy", 1);  // OGRE render-to-texture mode
    
    // In WSL, respect the force_raycast flag from SDF, but with a warning
    this->config.forceRaycast = _sdf->HasElement("force_raycast") ? 
                               _sdf->Get<bool>("force_raycast") : false;
    
    if (this->config.forceRaycast) {
      gzmsg << "[MmWaveSensorPlugin] Force raycast is enabled in WSL - will attempt ray casting" << std::endl;
    } else {
      gzmsg << "[MmWaveSensorPlugin] Force raycast is disabled - will use compatibility mode" << std::endl;
    }
    
    this->wslCompatMode = !this->config.forceRaycast;
  } else {
    // In non-WSL, check the force_raycast flag
    this->config.forceRaycast = _sdf->HasElement("force_raycast") ? 
                               _sdf->Get<bool>("force_raycast") : false;
    this->wslCompatMode = !this->config.forceRaycast;
  }
  
  gzmsg << "[MmWaveSensorPlugin] Mode settings: "
        << "usePointCloudsOnly=" << (this->config.usePointCloudsOnly ? "true" : "false") << ", "
        << "wslCompatMode=" << (this->wslCompatMode ? "true" : "false") << ", "
        << "forceRaycast=" << (this->config.forceRaycast ? "true" : "false") << std::endl;
  
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
  
  // Setup software rendering environment variables for WSL
  setenv("LIBGL_ALWAYS_SOFTWARE", "1", 1);
  setenv("MESA_GL_VERSION_OVERRIDE", "3.3", 1);
  
  // This is a WSL (Windows Subsystem for Linux) environment which is not yet fully compatible
  std::string wslenv = std::getenv("WSL_DISTRO_NAME") ? std::getenv("WSL_DISTRO_NAME") : "";
  if (!wslenv.empty()) {
    gzwarn << "[MmWaveSensorPlugin] Running in WSL environment: " << wslenv << ". Applying WSL compatibility settings." << std::endl;
    gzmsg << "[MmWaveSensorPlugin] LIBGL_ALWAYS_SOFTWARE=" << (std::getenv("LIBGL_ALWAYS_SOFTWARE") ? std::getenv("LIBGL_ALWAYS_SOFTWARE") : "not set") << std::endl;
    gzmsg << "[MmWaveSensorPlugin] MESA_GL_VERSION_OVERRIDE=" << (std::getenv("MESA_GL_VERSION_OVERRIDE") ? std::getenv("MESA_GL_VERSION_OVERRIDE") : "not set") << std::endl;
    // WSL detection already handled in Configure method
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

void MmWaveSensorPlugin::PostUpdate(const gz::sim::UpdateInfo &_info,
                                     const gz::sim::EntityComponentManager &_ecm)
{
    // Skip if the simulation is paused
    if (_info.paused)
        return;

    // Rate limit the sensor updates based on configured update rate
    auto currentTime = std::chrono::steady_clock::duration(_info.simTime);
    if ((currentTime - this->lastUpdateTime) < this->config.updatePeriod)
        return;
    this->lastUpdateTime = currentTime;

    // Safety check - immediately switch to compatibility mode if we had a rendering crash previously
    static int updateCounter = 0;
    updateCounter++;
    
    if (updateCounter == 5 && !this->wslCompatMode) {
        // On the 5th update, if still in ray casting mode and nothing fatal has happened,
        // check if we can actually perform ray casting safely
        try {
            // Simple test to see if we can access the rendering system safely
            bool canAccessRenderingSystem = gz::rendering::loadedEngines().size() > 0;
            if (!canAccessRenderingSystem) {
                // Don't immediately give up - attempt to load rendering engines manually
                gzmsg << "[MmWaveSensorPlugin] No rendering engines found during initial check. "
                      << "Will attempt to load them manually." << std::endl;
                
                try {
                    // Try to load OGRE first (more compatible)
                    auto ogreEngine = gz::rendering::engine("ogre");
                    if (ogreEngine) {
                        gzmsg << "[MmWaveSensorPlugin] Successfully loaded OGRE engine manually" << std::endl;
                        canAccessRenderingSystem = true;
                    }
                } catch (const std::exception& e) {
                    gzerr << "[MmWaveSensorPlugin] Error loading OGRE engine: " << e.what() << std::endl;
                }
                
                if (!canAccessRenderingSystem) {
                    try {
                        // Then try OGRE2
                        auto ogre2Engine = gz::rendering::engine("ogre2");
                        if (ogre2Engine) {
                            gzmsg << "[MmWaveSensorPlugin] Successfully loaded OGRE2 engine manually" << std::endl;
                            canAccessRenderingSystem = true;
                        }
                    } catch (const std::exception& e) {
                        gzerr << "[MmWaveSensorPlugin] Error loading OGRE2 engine: " << e.what() << std::endl;
                    }
                }
                
                // Only switch to compatibility mode if we still can't access rendering
                if (!canAccessRenderingSystem) {
                    if (!this->config.forceRaycast) {
                        gzwarn << "[MmWaveSensorPlugin] Failed to load any rendering engines. "
                              << "Switching to compatibility mode." << std::endl;
                        this->wslCompatMode = true;
                    } else {
                        gzwarn << "[MmWaveSensorPlugin] Failed to load rendering engines, but force_raycast=true. "
                              << "Will attempt to proceed anyway." << std::endl;
                    }
                } else {
                    gzmsg << "[MmWaveSensorPlugin] Successfully verified rendering system is available" << std::endl;
                }
            }
        } catch (const std::exception& e) {
            gzwarn << "[MmWaveSensorPlugin] Exception during rendering system check: "
                   << e.what() << ". Switching to compatibility mode." << std::endl;
            this->wslCompatMode = true;
        } catch (...) {
            gzwarn << "[MmWaveSensorPlugin] Unknown exception during rendering system check. "
                   << "Switching to compatibility mode." << std::endl;
            this->wslCompatMode = true;
        }
    }

    // Attempt scene acquisition for ray casting mode
    if (!this->wslCompatMode && !this->sceneManager->HasValidScene()) 
    {
        try {
            gzmsg << "[MmWaveSensorPlugin] Attempting to acquire scene for ray casting... Attempt #" 
                  << (this->sceneManager->GetAcquisitionAttempts() + 1) << std::endl;
                  
            bool acquired = this->sceneManager->AcquireScene();
            
            if (acquired) {
                gzmsg << "[MmWaveSensorPlugin] Successfully acquired rendering scene for ray casting!" << std::endl;
            }
            else if (this->sceneManager->GetAcquisitionAttempts() >= 5) {
                // Switch to compatibility mode after a few failed attempts
                gzmsg << "[MmWaveSensorPlugin] Failed to acquire scene after 5 attempts. "
                      << "Switching to compatibility mode." << std::endl;
                this->wslCompatMode = true;
            }
            else {
                gzmsg << "[MmWaveSensorPlugin] Scene acquisition failed, will try again" << std::endl;
            }
        } catch (const std::exception& e) {
            gzerr << "[MmWaveSensorPlugin] Exception during scene acquisition: " 
                  << e.what() << ". Switching to compatibility mode." << std::endl;
            this->wslCompatMode = true;
        } catch (...) {
            gzerr << "[MmWaveSensorPlugin] Unknown exception during scene acquisition. "
                  << "Switching to compatibility mode." << std::endl;
            this->wslCompatMode = true;
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
    try {
    msgs::PointCloudPacked pointCloudMsg = 
        this->messageHandler->CreatePointCloudMessage(_info.simTime.count(), sensorName);

    // ... (rest of the code remains the same)
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
            gzmsg << "[MmWaveSensorPlugin] Using WSL compatibility mode" << std::endl;
            
            // Update the simplified world model periodically
            auto now = std::chrono::steady_clock::now();
            if (now - this->lastWorldModelUpdate > this->config.worldModelUpdatePeriod)
            {
              olympus_sim::wsl_compat::UpdateSimplifiedWorldModel(
                  this->entity, _ecm, this->simplifiedWorldModel, this->wslWarningShown);
              this->lastWorldModelUpdate = now;
            }
            
            // Generate simulated data using the simplified world model
            olympus_sim::wsl_compat::GenerateSimulatedData(
                this->entity, worldSensorPose, _ecm, pointCloudMsg,
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
            
            gzmsg << "[MmWaveSensorPlugin] Generated point cloud in WSL mode with " 
                  << pointCloudMsg.width() << " points" << std::endl;
        }
        else {
            // APPROACH 3: Use normal ray query mode with full Gazebo rendering
            gzmsg << "[MmWaveSensorPlugin] Using ray casting simulation" << std::endl;
            
            // Initialize scene manager if not already done
            if (!this->sceneManager) {
                this->sceneManager = std::make_unique<MmWaveSensorScene>();
                gzmsg << "[MmWaveSensorPlugin] Created scene manager" << std::endl;
            }
            
            // Try to acquire the rendering scene
            if (!this->sceneManager->HasValidScene()) {
                gzmsg << "[MmWaveSensorPlugin] Attempting to acquire rendering scene... (attempt " 
                      << this->sceneManager->GetAcquisitionAttempts() + 1 << "/" 
                      << this->sceneManager->GetMaxAcquisitionAttempts() << ")" << std::endl;
                
                bool sceneAcquired = this->sceneManager->AcquireScene();
                
                if (!sceneAcquired) {
                    // Only fall back to WSL mode if not forcing raycast mode
                    if (!this->config.forceRaycast && !this->wslWarningShown) {
                        gzerr << "[MmWaveSensorPlugin] Failed to get rendering scene. "
                              << "Ray casting will not work." << std::endl;
                        
                        // Log environment variables for debugging
                        const char* libglSoftware = std::getenv("LIBGL_ALWAYS_SOFTWARE");
                        const char* wslDistro = std::getenv("WSL_DISTRO_NAME");
                        const char* mesaVersion = std::getenv("MESA_GL_VERSION_OVERRIDE");
                        gzmsg << "[MmWaveSensorPlugin] Environment variables: " 
                              << "LIBGL_ALWAYS_SOFTWARE=" << (libglSoftware ? libglSoftware : "not set") << ", "
                              << "WSL_DISTRO_NAME=" << (wslDistro ? wslDistro : "not set") << ", "
                              << "MESA_GL_VERSION_OVERRIDE=" << (mesaVersion ? mesaVersion : "not set") << std::endl;
                        
                        if (!this->config.forceRaycast) {
                            gzerr << "[MmWaveSensorPlugin] Falling back to WSL compatibility mode." << std::endl;
                            this->wslWarningShown = true;
                            this->wslCompatMode = true;
                        } else {
                            gzerr << "[MmWaveSensorPlugin] Will keep trying for ray casting due to force_raycast=true." << std::endl;
                        }
                    }
                    
                    // When force_raycast=true, try to manually create a rendering scene
                    if (this->config.forceRaycast) {
                        gzwarn << "[MmWaveSensorPlugin] Ray casting failed but force_raycast=true. Attempting manual scene creation" << std::endl;
                        
                        // Create a new scene manager with more aggressive options
                        if (!this->sceneManager->HasValidScene()) {
                            try {
                                // Set additional environment variables that might help
                                setenv("LIBGL_ALWAYS_SOFTWARE", "1", 1);
                                setenv("MESA_GL_VERSION_OVERRIDE", "3.3", 1);
                                setenv("LIBGL_ALWAYS_INDIRECT", "1", 1);
                                setenv("OGRE_RTT_MODE", "Copy", 1);
                                
                                gzmsg << "[MmWaveSensorPlugin] Set additional OpenGL environment variables for WSL compatibility" << std::endl;
                                
                                // Try to create a scene with any available engine
                                auto availableEngines = gz::rendering::loadedEngines();
                                gzmsg << "[MmWaveSensorPlugin] Available engines after setting env vars: " << availableEngines.size() << std::endl;
                                
                                for (const auto& engineName : availableEngines) {
                                    gzmsg << "[MmWaveSensorPlugin] Found engine: " << engineName << std::endl;
                                }
                                
                                // Try one more scene acquisition with new environment variables
                                bool forcedAcquisition = this->sceneManager->ForceAcquireScene();
                                if (forcedAcquisition) {
                                    gzmsg << "[MmWaveSensorPlugin] Successfully forced scene acquisition!" << std::endl;
                                    return; // Try again on the next update
                                }
                            } catch (const std::exception& e) {
                                gzerr << "[MmWaveSensorPlugin] Exception during forced scene acquisition: " << e.what() << std::endl;
                            }
                            
                            // If all else fails, add a test point to keep the pipeline running
                            gzwarn << "[MmWaveSensorPlugin] Adding test points since all ray casting attempts failed" << std::endl;
                            this->messageHandler->AddTestPoint(pointCloudMsg);
                            this->pointCloudPublisher.Publish(pointCloudMsg);
                        }
                    }
                    
                    // Skip the rest of the ray casting logic
                    return;
                } else {
                    gzmsg << "[MmWaveSensorPlugin] Successfully acquired rendering scene" << std::endl;
                }
            }
            
            // Check if scene manager is ready (has both scene and ray query)
            if (!this->sceneManager->IsReady()) {
                gzerr << "[MmWaveSensorPlugin] Scene manager is not ready for ray casting" << std::endl;
                return;
            }
            
            // Create ray sensor object if needed
            if (!this->raySensor) {
                this->raySensor = std::make_unique<MmWaveSensorRay>();
                gzmsg << "[MmWaveSensorPlugin] Created ray sensor" << std::endl;
            }
            
            // Set the ray query from the scene manager
            this->raySensor->SetRayQuery(this->sceneManager->GetRayQuery());
            gzmsg << "[MmWaveSensorPlugin] Ray sensor initialized with ray query" << std::endl;
            
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
        
        // Minimal logging every 50 updates
        static int updateCounter = 0;
        if (++updateCounter % 50 == 0) {
            gzdbg << "[MmWaveSensorPlugin] Points: " << pointCloudMsg.width() << std::endl;
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
