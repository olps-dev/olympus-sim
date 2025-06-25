#include "raycast/MmWaveSensorScene.hh"
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/Scene.hh>

using namespace olympus_sim;

MmWaveSensorScene::MmWaveSensorScene()
{
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
    return false;
  }
  
  // Try to get a rendering scene
  // Get the list of loaded engines
  auto loadedEngines = gz::rendering::loadedEngines();
  if (loadedEngines.empty())
  {
    if (this->sceneAcquisitionAttempts == 1 || 
        this->sceneAcquisitionAttempts == this->maxSceneAcquisitionAttempts)
    {
      gzwarn << "[MmWaveSensorScene] No rendering engines loaded." << std::endl;
    }
    return false;
  }
  
  // Try each engine
  for (const auto &engineName : loadedEngines)
  {
    if (engineName.empty())
      continue;
      
    gzdbg << "[MmWaveSensorScene] Checking engine '" << engineName << "'" << std::endl;
    
    // Get the engine instance
    auto engine = gz::rendering::engine(engineName);
    if (!engine)
      continue;
    
    // Get scenes from this engine
    unsigned int sceneCount = engine->SceneCount();
    for (unsigned int i = 0; i < sceneCount; ++i)
    {
      auto renderingScene = engine->SceneByIndex(i);
      if (renderingScene)
      {
        gzdbg << "[MmWaveSensorScene] Found scene '" << renderingScene->Name() << "'" << std::endl;
        if (renderingScene)
        {
          gzmsg << "[MmWaveSensorScene] Successfully acquired scene '" 
                << renderingScene->Name() << "' on attempt " 
                << this->sceneAcquisitionAttempts << std::endl;
                
          // Store the scene
          this->scene = renderingScene;
          
          // Create new ray query for the scene with better error handling
          try {
            this->rayQuery = this->scene->CreateRayQuery();
            if (this->rayQuery) {
              gzdbg << "[MmWaveSensorScene] Successfully created RayQuery" << std::endl;
              return true;
            } else {
              gzerr << "[MmWaveSensorScene] Failed to create RayQuery - null pointer returned" << std::endl;
            }
          } catch (const std::exception& e) {
            gzerr << "[MmWaveSensorScene] Exception creating RayQuery: " << e.what() << std::endl;
          }
        }
      }
    }
  }
  
  // Log failure periodically to avoid spamming
  if (this->sceneAcquisitionAttempts % 10 == 0 || 
      this->sceneAcquisitionAttempts == this->maxSceneAcquisitionAttempts) 
  {
    gzwarn << "[MmWaveSensorScene] Still trying to find rendering scene "
           << "(attempt " << this->sceneAcquisitionAttempts << "" 
           << (this->sceneAcquisitionAttempts == this->maxSceneAcquisitionAttempts ? 
              ", FINAL ATTEMPT" : "") 
           << "). This is normal in some environments." << std::endl;
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
