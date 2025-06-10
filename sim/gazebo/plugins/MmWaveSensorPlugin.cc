#include "MmWaveSensorPlugin.hh"

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
    // Do not return yet, try to initialize rendering components
  }

  // Check if running in WSL environment - this affects rendering behavior
  char* wslEnv = std::getenv("WSL_DISTRO_NAME");
  bool isRunningInWSL = (wslEnv != nullptr);
  if (isRunningInWSL) {
    gzwarn << "[" << pluginName << "] Running in WSL environment: " << wslEnv << "\n"
           << "OpenGL rendering may have limitations. Will attempt to work in degraded mode if needed." << std::endl;
  }

  // Defer scene acquisition to PostUpdate to give rendering system more time to initialize
  // This is especially important for WSL environments where rendering is slower to initialize
  gzdbg << "[" << pluginName << "] Deferring scene acquisition and RayQuery initialization to PostUpdate.";
  // Do not attempt to acquire scene here - will be done in PostUpdate
}

//////////////////////////////////////////////////
// Helper methods implementation
//////////////////////////////////////////////////

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
      gzdbg << "[MmWaveSensorPlugin::Scene] WSL detected, will use all available scene finding strategies" << std::endl;
    }
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
  
  // Do NOT create a new scene as it would be empty and causes crashes in WSL
  gzwarn << "[MmWaveSensorPlugin] Failed to find an existing scene.\n"
         << "Ray casting will not work until a scene is available.\n"
         << "The plugin will keep trying in PostUpdate().\n";
  
  // In WSL/WSLg, it often takes time for the rendering scene to be available
  // Rather than creating an empty scene, return nullptr and let PostUpdate() retry
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
                                                 const gz::math::Pose3d &_sensorPose, // Added sensor pose
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

void MmWaveSensorPlugin::CastRays(const gz::math::Pose3d &_sensorPose,
                         const gz::sim::EntityComponentManager &_ecm,
                         gz::msgs::PointCloudPacked &_pointCloudMsg)
{
  // Debug message about sensor position
  gzdbg << "[MmWaveSensorPlugin] Sensor pose: pos=" << _sensorPose.Pos() 
        << ", rot=" << _sensorPose.Rot().Euler() << std::endl;

  if (!this->scene || !this->rayQuery)
  {
    gzerr << "[MmWaveSensorPlugin] Ray casting unavailable - scene or rayQuery missing" << std::endl;
    _pointCloudMsg.set_width(0);
    _pointCloudMsg.set_height(0);
    _pointCloudMsg.set_row_step(0);
    _pointCloudMsg.set_data("");
    _pointCloudMsg.set_is_dense(true);
    return;
  }
  
  // Try to reload the scene to ensure synchronization with physics
  try {
    // Force scene update to sync with latest physics state
    this->scene->PreRender();
    gzdbg << "[MmWaveSensorPlugin] Scene pre-render completed" << std::endl;
  } catch (const std::exception &e) {
    gzerr << "[MmWaveSensorPlugin] Exception during scene update: " << e.what() << std::endl;
  } catch (...) {
    gzerr << "[MmWaveSensorPlugin] Unknown exception during scene update" << std::endl;
  }
  
  // RayQueryType::CLOSEST is implied by using ClosestPoint().
  // Print out sensor parameters
  gzdbg << "[MmWaveSensorPlugin] Sensor parameters: maxRange=" << this->maxRange 
        << ", horizontalFov=" << this->horizontalFov << ", verticalFov=" << this->verticalFov << std::endl;

  // Define test directions with increased range to ensure detection
  const double EXTENDED_RANGE_FACTOR = 1.5; // Use 1.5x the configured range for testing
  const double testRange = this->maxRange * EXTENDED_RANGE_FACTOR;
  
  std::vector<std::pair<gz::math::Vector3d, std::string>> directions = {
    {{1.0, 0.0, 0.0}, "forward (to box)"},        // Forward - should hit the box at x=5
    {{0.0, 1.0, 0.0}, "right (to sphere)"},       // Right - might hit sphere
    {{0.0, -1.0, 0.0}, "left"},                  // Left
    {{0.0, 0.0, -1.0}, "down (to ground)"},       // Down - should hit ground
    {{0.707, 0.707, 0.0}, "forward-right"}        // 45 degrees right
  };

  // Additional directions with small angular offsets to increase hit probability
  for (double angle = -0.2; angle <= 0.2; angle += 0.1) {
    if (angle == 0.0) continue; // Skip exact 0 as we already have it
    
    // Add slightly angled forward rays
    directions.push_back({
      {1.0, angle, 0.0},
      "forward-angled (" + std::to_string(angle) + ")"
    });
    
    // Add slightly angled downward rays
    directions.push_back({
      {0.0, 0.0, -1.0 + angle},
      "down-angled (" + std::to_string(angle) + ")"
    });
  }

  int hitCount = 0;
  std::vector<gz::math::Vector3d> hitPoints;
  
  try {
    // Try multiple directions
    for (const auto& dirPair : directions) {
      const auto& dir = dirPair.first.Normalized(); // Ensure direction is normalized
      const auto& dirName = dirPair.second;
      
      // Cast ray in world frame - handle coordinate transformations properly
      gz::math::Vector3d worldDir = _sensorPose.Rot().RotateVector(dir);
      gz::math::Vector3d origin = _sensorPose.Pos(); // Use world position as origin
      
      gzdbg << "[MmWaveSensorPlugin] Casting ray " << dirName 
            << ": from " << origin << " in direction " << worldDir << std::endl;

      // Try two different methods to maximize chances of success
      // Method 1: Use the RayQuery directly with scene coordinates
      this->rayQuery->SetOrigin(origin);
      this->rayQuery->SetDirection(worldDir);
      // SetMaxDistance is not available on RayQuery in Gazebo Harmonic.
      // Distance filtering will be done after getting the result.
      gz::rendering::RayQueryResult result = this->rayQuery->ClosestPoint();
      
      if (result) {
        // Ray hit something - get details and save hit point
        gz::math::Vector3d hitPoint = origin + worldDir * result.distance;
        hitPoints.push_back(hitPoint);
        hitCount++;
        
        if (result.distance <= this->maxRange) {
          gzmsg << "[MmWaveSensorPlugin] HIT: Ray " << dirName << " hit at distance: " 
                << result.distance << " m (within maxRange: " << this->maxRange 
                << " m), object ID: " << result.objectId 
                << ", hit point: " << hitPoint << std::endl;
        } else {
          gzdbg << "[MmWaveSensorPlugin] HIT (beyond range): Ray " << dirName 
                << " hit at distance: " << result.distance << " m (exceeds maxRange: " 
                << this->maxRange << " m), object ID: " << result.objectId 
                << ", hit point: " << hitPoint << std::endl;
        }
      } else {
        // Method 2: Try an alternative approach with segmented ray tests
        bool hitDetected = false;
        
        // Try testing with segmented rays at different distances
        const int numSegments = 10;
        for (int i = 1; i <= numSegments; i++) {
          double segmentDist = (testRange * i) / numSegments;
          gz::math::Vector3d endPoint = origin + worldDir * segmentDist;
          
          // Segment test with direct line test
          this->rayQuery->SetOrigin(origin);
          this->rayQuery->SetDirection((endPoint - origin).Normalized());
          // SetMaxDistance is not available on RayQuery in Gazebo Harmonic.
          // Distance filtering will be done after getting the result.
          result = this->rayQuery->ClosestPoint();
          if (result) {
            // Hit detected with segmented approach
            gz::math::Vector3d hitPoint = origin + worldDir * result.distance;
            hitPoints.push_back(hitPoint);
            hitCount++;
            hitDetected = true;
            
            gzmsg << "[MmWaveSensorPlugin] HIT (segmented): Ray " << dirName 
                  << " hit at segment " << i << "/" << numSegments 
                  << ", distance: " << result.distance << " m, object ID: " 
                  << result.objectId << ", hit point: " << hitPoint << std::endl;
            break;
          }
        }
        
        if (!hitDetected) {
          // No hit with either method
          gzdbg << "[MmWaveSensorPlugin] MISS: Ray " << dirName << " did not hit anything" << std::endl;
          gz::math::Vector3d endPoint = origin + worldDir * this->maxRange;
          gzdbg << "[MmWaveSensorPlugin] Ray end point (at max range): " << endPoint << std::endl;
        }
      }
    }
  } catch (const std::exception &e) {
    gzerr << "[MmWaveSensorPlugin] Exception during ray cast: " << e.what() << std::endl;
  } catch (...) {
    gzerr << "[MmWaveSensorPlugin] Unknown exception during ray cast" << std::endl;
  }

  // Only add test point if no real hits were detected
  if (hitCount == 0) {
    gzwarn << "[MmWaveSensorPlugin] No points detected by ray casting, adding test point" << std::endl;
    _pointCloudMsg.set_width(0);
    _pointCloudMsg.set_height(0);
    _pointCloudMsg.set_row_step(0);
    _pointCloudMsg.set_data("");
    _pointCloudMsg.set_is_dense(true);
  } else {
    gzmsg << "[MmWaveSensorPlugin] Detected " << hitCount << " hit points" << std::endl;
    
    // Convert hit points to point cloud message
    // Code to populate point cloud message with real hit points would go here
    // For now, just leave as empty to isolate testing of ray casting
  }
}

//////////////////////////////////////////////////
void MmWaveSensorPlugin::PostUpdate(const gz::sim::UpdateInfo &_info,
                                     const gz::sim::EntityComponentManager &_ecm)
{
    // Skip updates if not enough time has passed based on update rate
    // Convert simulation time to std::chrono for comparison
    std::chrono::steady_clock::duration simTime = 
        std::chrono::nanoseconds(static_cast<int64_t>(_info.simTime.count() * 1e9));
    
    if ((simTime - this->lastUpdateTime) < this->updatePeriod)
        return;
    
    // Update last update time
    this->lastUpdateTime = simTime;
    
    // Wait for rendering to be properly initialized
    // In WSL/WSLg, rendering scenes may take longer to become available
    static int retryCount = 0;
    static auto lastSceneAttemptTime = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    
    // Check if running in WSL to adjust retry strategy
    static bool isWSL = false;
    static bool checkedWSL = false;
    if (!checkedWSL) {
        char* wslEnv = std::getenv("WSL_DISTRO_NAME");
        isWSL = (wslEnv != nullptr);
        checkedWSL = true;
    }
    
    // In WSL, rendering often takes longer to initialize, so we use a more 
    // aggressive retry strategy early on, then back off over time
    std::chrono::seconds retryInterval;
    int maxRetries = isWSL ? 60 : 30;  // More retries in WSL
    
    // Dynamic retry interval strategy
    if (retryCount < 10) {
        // Very frequent early retries
        retryInterval = isWSL ? std::chrono::seconds(1) : std::chrono::seconds(2);
    } else if (retryCount < 20) {
        retryInterval = std::chrono::seconds(3);
    } else if (retryCount < maxRetries) {
        retryInterval = std::chrono::seconds(5);
    } else {
        retryInterval = std::chrono::seconds(10);
    }
    
    // Try to get scene if we don't have one yet, or retry periodically with a maximum attempt count
    if ((!this->scene || ((now - lastSceneAttemptTime) > retryInterval)) && retryCount < maxRetries) 
    {
        lastSceneAttemptTime = now;
        retryCount++;
        
        // More detailed logging for the early attempts and WSL environments
        if ((retryCount <= 5) || (retryCount % 5 == 0) || (isWSL && retryCount <= 10)) {
            gzdbg << "[MmWaveSensorPlugin::PostUpdate] Attempt #" << retryCount 
                  << " to acquire rendering scene" 
                  << (isWSL ? " (WSL environment)" : "") << "..." << std::endl;
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
        else if (retryCount % 10 == 0 || retryCount == maxRetries) {
            // Log periodically to avoid spamming, but ensure last attempt is always logged
            gzwarn << "[MmWaveSensorPlugin::PostUpdate] Still trying to find rendering scene "
                   << "(attempt " << retryCount << "" 
                   << (retryCount == maxRetries ? ", FINAL ATTEMPT" : "") 
                   << "). This is normal in WSL/WSLg." << std::endl;
        }
    }
    
    // We don't need this section anymore as ray query creation is now handled above
    // when we successfully find a scene

    // If scene or rayQuery is still null, log error and try again next update
    if (!this->scene || !this->rayQuery)
    {
        static int error_count = 0;
        if (error_count % 100 == 0) { // Log periodically
             gzwarn << "[MmWaveSensorPlugin::PostUpdate] Scene or RayQuery unavailable. "
                    << "Will keep trying." << std::endl;
        }
        error_count++;
        return;
    }

  if (_info.simTime < this->lastUpdateTime + this->updatePeriod && _info.simTime > std::chrono::seconds(0)) {
    return;
  }
  this->lastUpdateTime = _info.simTime;

  // Skip ray-casting if scene or rayQuery aren't available (common in WSL)
  if (!this->scene || !this->rayQuery) {
    // Print warning message once per second to avoid console flooding
    static auto lastWarningTime = std::chrono::steady_clock::now();
    auto currentTime = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(currentTime - lastWarningTime).count();
    
    if (elapsed >= 5) { // Only warn every 5 seconds
      gzdbg << "[MmWaveSensorPlugin] Skipping update - ray casting not available in this environment.\n";
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
  math::Pose3d sensorPose = poseComp->Data();

  // Debug - Entity hierarchy and pose information
  gzmsg << "[MmWaveSensorPlugin] Current entity ID: " << this->entity << "\n";

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
      math::Pose3d parentPose = parentPoseComp->Data();
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
        math::Pose3d grandparentPose = grandparentPoseComp->Data();
        gzmsg << "[MmWaveSensorPlugin] Grandparent entity pose: " << grandparentPose << "\n";
      }
    }
  }
  
  // Instead of just using the entity's pose, try to get the world pose
  math::Pose3d worldSensorPose = gz::sim::worldPose(this->entity, _ecm);
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
    // Cast rays to generate point cloud - use world pose instead of local pose
    this->CastRays(worldSensorPose, _ecm, pointCloudMsg);
    
    // Publish point cloud message
    this->pointCloudPublisher.Publish(pointCloudMsg);
  }
  catch (const std::exception& e) {
    gzerr << "[MmWaveSensorPlugin] Exception during ray casting: " << e.what() << std::endl;
  }
  catch (...) {
    gzerr << "[MmWaveSensorPlugin] Unknown exception during ray casting" << std::endl;
  }

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
