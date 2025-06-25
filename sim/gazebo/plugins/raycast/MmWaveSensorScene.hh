#ifndef OLYMPUS_SIM_MMWAVE_SENSOR_SCENE_HH_
#define OLYMPUS_SIM_MMWAVE_SENSOR_SCENE_HH_

#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/RayQuery.hh>
#include <gz/common/Console.hh>

namespace olympus_sim
{

/**
 * @brief Class to handle scene management for MmWave sensor plugin
 * 
 * This class abstracts the scene acquisition and management logic
 */
class MmWaveSensorScene
{
public:
  /**
   * @brief Constructor
   */
  MmWaveSensorScene();

  /**
   * @brief Try to acquire the rendering scene
   * @return True if scene acquired or already available, false otherwise
   */
  bool AcquireScene();

  /**
   * @brief Get the rendering scene pointer
   * @return The rendering scene pointer or nullptr if not available
   */
  gz::rendering::ScenePtr GetScene() const;

  /**
   * @brief Get the ray query object for the scene
   * @return The ray query object or nullptr if not available
   */
  gz::rendering::RayQueryPtr GetRayQuery() const;

  /**
   * @brief Check if scene and ray query are available
   * @return True if both scene and ray query are available, false otherwise
   */
  bool IsReady() const;

  /**
   * @brief Check if we have a valid scene
   * @return True if scene is valid, false otherwise
   */
  bool HasValidScene() const;

  /**
   * @brief Get number of attempts made to acquire the scene
   * @return Number of acquisition attempts
   */
  int GetAcquisitionAttempts() const;

  /**
   * @brief Get maximum number of attempts to acquire the scene
   * @return Maximum number of acquisition attempts
   */
  int GetMaxAcquisitionAttempts() const;

private:
  // Rendering scene
  gz::rendering::ScenePtr scene{nullptr};

  // Ray query interface for ray casting
  gz::rendering::RayQueryPtr rayQuery{nullptr};

  // Count the number of attempts to acquire the scene
  int sceneAcquisitionAttempts{0};
  const int maxSceneAcquisitionAttempts{5};
};

}  // namespace olympus_sim

#endif  // OLYMPUS_SIM_MMWAVE_SENSOR_SCENE_HH_
