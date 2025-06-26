#include "raycast/MmWaveSensorScene.hh"
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/Scene.hh>
#include <cstdlib>

using namespace olympus_sim;

MmWaveSensorScene::MmWaveSensorScene()
{
  // Force software rendering for WSL
  setenv("LIBGL_ALWAYS_SOFTWARE", "1", 1);
  
  // These environment variables can help with WSL graphics issues
  setenv("MESA_GL_VERSION_OVERRIDE", "3.3", 1);
  
  // Try to set some additional environment variables for WSL rendering
  setenv("GZ_RENDERING_ENGINE", "ogre", 1);  // Try to ensure OGRE is used
  setenv("GZ_OGRE_RENDERSYSTEM", "GL", 1);  // Force OpenGL render system
  
  // Print environment variables to help with debugging
  gzmsg << "[MmWaveSensorScene] Environment variables:" << std::endl;
  gzmsg << "  LIBGL_ALWAYS_SOFTWARE=" << (std::getenv("LIBGL_ALWAYS_SOFTWARE") ? std::getenv("LIBGL_ALWAYS_SOFTWARE") : "not set") << std::endl;
  gzmsg << "  MESA_GL_VERSION_OVERRIDE=" << (std::getenv("MESA_GL_VERSION_OVERRIDE") ? std::getenv("MESA_GL_VERSION_OVERRIDE") : "not set") << std::endl;
  gzmsg << "  GZ_RENDERING_ENGINE=" << (std::getenv("GZ_RENDERING_ENGINE") ? std::getenv("GZ_RENDERING_ENGINE") : "not set") << std::endl;
  gzmsg << "  GZ_OGRE_RENDERSYSTEM=" << (std::getenv("GZ_OGRE_RENDERSYSTEM") ? std::getenv("GZ_OGRE_RENDERSYSTEM") : "not set") << std::endl;
  gzmsg << "  WSL_DISTRO_NAME=" << (std::getenv("WSL_DISTRO_NAME") ? std::getenv("WSL_DISTRO_NAME") : "not set") << std::endl;
  
  // Increase max scene acquisition attempts for WSL environments
  this->maxSceneAcquisitionAttempts = 50;
}

bool MmWaveSensorScene::AcquireScene()
{
  // Skip if we already have a valid scene and ray query
  if (this->scene && this->rayQuery)
    return true;

  // Limit the number of attempts to avoid infinite retries
  this->sceneAcquisitionAttempts++;
  if (this->sceneAcquisitionAttempts > this->maxSceneAcquisitionAttempts)
  {
    gzerr << "[MmWaveSensorScene] Failed to acquire scene after " 
          << this->maxSceneAcquisitionAttempts << " attempts." << std::endl;
    return false;
  }
  
  // Check if any rendering engines are loaded
  if (gz::rendering::loadedEngines().empty())
  {
    gzmsg << "[MmWaveSensorScene] No rendering engines loaded. Will continue trying." << std::endl;
    // We can't load the engine directly - it should already be loaded by Gazebo
  }
  
  // Try to get a rendering scene
  // Get the list of loaded engines
  auto loadedEngines = gz::rendering::loadedEngines();
  if (loadedEngines.empty())
  {
    if (this->sceneAcquisitionAttempts == 1 || 
        this->sceneAcquisitionAttempts % 5 == 0)
    {
      gzwarn << "[MmWaveSensorScene] No rendering engines loaded after explicit loading attempt." << std::endl;
      
      // Attempt to manually load engines as a last resort
      gzmsg << "[MmWaveSensorScene] Attempting to manually load rendering engines..." << std::endl;
      
      // Try to load OGRE first (most compatible)
      try {
        gzmsg << "[MmWaveSensorScene] Attempting to load OGRE rendering engine..." << std::endl;
        auto ogreEngine = gz::rendering::engine("ogre");
        if (ogreEngine) {
          gzmsg << "[MmWaveSensorScene] Successfully loaded OGRE engine" << std::endl;
        }
      } catch (const std::exception& e) {
        gzerr << "[MmWaveSensorScene] Error loading OGRE engine: " << e.what() << std::endl;
      }
      
      // Then try OGRE2 if OGRE didn't work
      try {
        gzmsg << "[MmWaveSensorScene] Attempting to load OGRE2 rendering engine..." << std::endl;
        auto ogre2Engine = gz::rendering::engine("ogre2");
        if (ogre2Engine) {
          gzmsg << "[MmWaveSensorScene] Successfully loaded OGRE2 engine" << std::endl;
        }
      } catch (const std::exception& e) {
        gzerr << "[MmWaveSensorScene] Error loading OGRE2 engine: " << e.what() << std::endl;
      }
      
      // Get the updated list of engines after our loading attempts
      loadedEngines = gz::rendering::loadedEngines();
      if (!loadedEngines.empty()) {
        gzmsg << "[MmWaveSensorScene] Successfully loaded rendering engines after manual attempt" << std::endl;
      }
    }
    
    if (loadedEngines.empty()) {
      return false;
    }
  }
  
  gzmsg << "[MmWaveSensorScene] Found " << loadedEngines.size() << " rendering engine(s)" << std::endl;
  for (const auto &engineName : loadedEngines) {
    gzmsg << "  - Engine: '" << (engineName.empty() ? "[unnamed]" : engineName) << "'" << std::endl;
  }
  
  // Try each engine
  for (const auto &engineName : loadedEngines)
  {
    if (engineName.empty())
      continue;
      
    gzmsg << "[MmWaveSensorScene] Checking engine '" << engineName << "'" << std::endl;
    
    // Get the engine instance
    auto engine = gz::rendering::engine(engineName);
    if (!engine)
    {
      gzmsg << "[MmWaveSensorScene] Failed to get engine instance for '" << engineName << "'" << std::endl;
      continue;
    }
    
    gzmsg << "[MmWaveSensorScene] Engine '" << engineName << "' has " 
          << engine->SceneCount() << " scene(s)" << std::endl;
    
    // Get scenes from this engine
    unsigned int sceneCount = engine->SceneCount();
    for (unsigned int i = 0; i < sceneCount; ++i)
    {
      auto renderingScene = engine->SceneByIndex(i);
      if (!renderingScene)
      {
        gzmsg << "[MmWaveSensorScene] Scene at index " << i << " is null" << std::endl;
        continue;
      }
      
      gzmsg << "[MmWaveSensorScene] Found scene '" << renderingScene->Name() << "'" << std::endl;
      
      // Store the scene
      this->scene = renderingScene;
      
      // Create new ray query for the scene with better error handling
      try {
        gzmsg << "[MmWaveSensorScene] Attempting to create RayQuery for scene '" << renderingScene->Name() << "'..." << std::endl;
        
        // Verify scene is initialized and valid
        if (this->scene->IsInitialized()) {
          gzmsg << "[MmWaveSensorScene] Scene is initialized" << std::endl;
        } else {
          gzwarn << "[MmWaveSensorScene] Scene is not initialized, trying anyway..." << std::endl;
        }
        
        // Try to get rendering properties to verify scene is working
        gzmsg << "[MmWaveSensorScene] Scene has " << this->scene->NodeCount() << " nodes" << std::endl;
        
        // Create the ray query
        this->rayQuery = this->scene->CreateRayQuery();
        
        if (this->rayQuery) {
          gzmsg << "[MmWaveSensorScene] Successfully created RayQuery!" << std::endl;
          
          // Perform a simple test ray cast to verify that the ray query works
          gz::math::Vector3d origin(0, 0, 0);
          gz::math::Vector3d direction(1, 0, 0);
          this->rayQuery->SetOrigin(origin);
          this->rayQuery->SetDirection(direction);
          auto testResult = this->rayQuery->ClosestPoint();
          
          gzmsg << "[MmWaveSensorScene] Test ray cast: valid=" << (testResult ? "true" : "false")
                << ", distance=" << testResult.distance << std::endl;
          
          return true;
        } else {
          gzerr << "[MmWaveSensorScene] Failed to create RayQuery - null pointer returned" << std::endl;
        }
      } catch (const std::exception& e) {
        gzerr << "[MmWaveSensorScene] Exception creating RayQuery: " << e.what() << std::endl;
      } catch (...) {
        gzerr << "[MmWaveSensorScene] Unknown exception creating RayQuery" << std::endl;
      }
    }
    
    // If no existing scenes were found, try creating a new scene
    if (sceneCount == 0)
    {
      gzmsg << "[MmWaveSensorScene] No existing scenes found. Attempting to create new scene..." << std::endl;
      
      try {
        // Create a new scene with a unique name
        std::string sceneName = "mmwave_ray_scene_" + std::to_string(this->sceneAcquisitionAttempts);
        this->scene = engine->CreateScene(sceneName);
        
        if (!this->scene)
        {
          gzerr << "[MmWaveSensorScene] Failed to create new scene" << std::endl;
          continue;
        }
        
        gzmsg << "[MmWaveSensorScene] Successfully created new scene '" << sceneName << "'" << std::endl;
        
        // Initialize the scene
        this->scene->SetAmbientLight(1.0, 1.0, 1.0, 1.0);
        
        // Create ray query
        this->rayQuery = this->scene->CreateRayQuery();
        if (this->rayQuery)
        {
          gzmsg << "[MmWaveSensorScene] Successfully created RayQuery for new scene" << std::endl;
          return true;
        }
        else
        {
          gzerr << "[MmWaveSensorScene] Failed to create RayQuery for new scene" << std::endl;
        }
      }
      catch (const std::exception& e)
      {
        gzerr << "[MmWaveSensorScene] Exception creating new scene: " << e.what() << std::endl;
      }
      catch (...)
      {
        gzerr << "[MmWaveSensorScene] Unknown exception creating new scene" << std::endl;
      }
    }
  }
  
  // Log failure periodically to avoid spamming
  if (this->sceneAcquisitionAttempts % 5 == 0 || 
      this->sceneAcquisitionAttempts == this->maxSceneAcquisitionAttempts) 
  {
    gzwarn << "[MmWaveSensorScene] Still trying to find rendering scene "
           << "(attempt " << this->sceneAcquisitionAttempts << "/" 
           << this->maxSceneAcquisitionAttempts << ")." << std::endl;
    
    // Log environment variables to help with debugging
    const char* libglSoftware = std::getenv("LIBGL_ALWAYS_SOFTWARE");
    const char* mesaVersion = std::getenv("MESA_GL_VERSION_OVERRIDE");
    gzmsg << "[MmWaveSensorScene] Environment variables: " 
          << "LIBGL_ALWAYS_SOFTWARE=" << (libglSoftware ? libglSoftware : "not set") 
          << ", MESA_GL_VERSION_OVERRIDE=" << (mesaVersion ? mesaVersion : "not set") << std::endl;
  }
  
  return false;
}

gz::rendering::ScenePtr MmWaveSensorScene::GetScene() const
{
  return this->scene;
}

gz::rendering::RayQueryPtr MmWaveSensorScene::GetRayQuery() const
{
  return this->rayQuery;
}

bool MmWaveSensorScene::IsReady() const
{
  return (this->scene != nullptr && this->rayQuery != nullptr);
}

bool MmWaveSensorScene::HasValidScene() const
{
  return this->scene != nullptr;
}

int MmWaveSensorScene::GetAcquisitionAttempts() const
{
  return this->sceneAcquisitionAttempts;
}

int MmWaveSensorScene::GetMaxAcquisitionAttempts() const
{
  return this->maxSceneAcquisitionAttempts;
}

bool MmWaveSensorScene::ForceAcquireScene()
{
  gzmsg << "[MmWaveSensorScene] Attempting FORCED scene acquisition for WSL environment..." << std::endl;
  
  // Set even more aggressive environment variables for graphics in WSL
  setenv("LIBGL_ALWAYS_SOFTWARE", "1", 1);
  setenv("MESA_GL_VERSION_OVERRIDE", "3.3", 1);
  setenv("LIBGL_ALWAYS_INDIRECT", "1", 1);
  setenv("OGRE_RTT_MODE", "Copy", 1);
  setenv("GALLIUM_DRIVER", "llvmpipe", 1); // Force software rendering using llvmpipe
  
  // Reset acquisition attempts to try again
  this->sceneAcquisitionAttempts = 0;
  
  // Try different rendering engine loaders
  std::vector<std::string> engineNames = {"ogre", "ogre2"};
  
  for (const auto& engineName : engineNames)
  {
    try {
      gzmsg << "[MmWaveSensorScene] Aggressively loading " << engineName << " engine..." << std::endl;
      
      // Force engine load
      auto engine = gz::rendering::engine(engineName);
      if (!engine)
      {
        gzerr << "[MmWaveSensorScene] Failed to load " << engineName << " engine" << std::endl;
        continue;
      }
      
      gzmsg << "[MmWaveSensorScene] Successfully loaded " << engineName << " engine" << std::endl;
      
      // Try to create a new scene if no scenes exist
      if (engine->SceneCount() == 0)
      {
        gzmsg << "[MmWaveSensorScene] No scenes found, creating new scene..." << std::endl;
        try {
          auto newScene = engine->CreateScene("mmwave_forced_scene");
          if (newScene)
          {
            gzmsg << "[MmWaveSensorScene] Created new scene successfully" << std::endl;
            this->scene = newScene;
            
            // Initialize scene
            if (!this->scene->IsInitialized())
            {
              gzmsg << "[MmWaveSensorScene] Initializing new scene..." << std::endl;
              this->scene->SetAmbientLight(0.3, 0.3, 0.3);
              this->scene->SetBackgroundColor(0.0, 0.0, 0.0);
            }
            
            // Create ray query
            try {
              this->rayQuery = this->scene->CreateRayQuery();
              if (this->rayQuery)
              {
                gzmsg << "[MmWaveSensorScene] Successfully created ray query" << std::endl;
                return true;
              }
              else
              {
                gzerr << "[MmWaveSensorScene] Failed to create ray query for new scene" << std::endl;
              }
            }
            catch (const std::exception& e)
            {
              gzerr << "[MmWaveSensorScene] Exception creating ray query: " << e.what() << std::endl;
            }
          }
        }
        catch (const std::exception& e)
        {
          gzerr << "[MmWaveSensorScene] Exception creating scene: " << e.what() << std::endl;
        }
      }
      else
      {
        // Try the normal acquisition approach with more aggressive retry
        if (AcquireScene())
        {
          return true;
        }
      }
    }
    catch (const std::exception& e)
    {
      gzerr << "[MmWaveSensorScene] Exception during forced scene acquisition with " 
            << engineName << ": " << e.what() << std::endl;
    }
    catch (...)
    {
      gzerr << "[MmWaveSensorScene] Unknown exception during forced scene acquisition with " 
            << engineName << std::endl;
    }
  }
  
  // If we got here, all attempts failed
  gzerr << "[MmWaveSensorScene] All forced scene acquisition attempts failed" << std::endl;
  return false;
}
