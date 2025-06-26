#ifndef OLYMPUS_SIM_MMWAVE_SENSOR_CONFIG_HH_
#define OLYMPUS_SIM_MMWAVE_SENSOR_CONFIG_HH_

#include <string>
#include <memory>
#include <chrono>
#include <sdf/Element.hh>
#include <gz/common/Console.hh>

namespace olympus_sim
{

/**
 * @brief Configuration class for MmWave sensor plugin
 * 
 * This class separates configuration parsing and storage from the main plugin logic
 */
class MmWaveSensorConfig
{
public:
  /**
   * @brief Load configuration from SDF element
   * @param _sdf The SDF element containing configuration
   * @param _pluginName Name of the plugin for logging
   * @return True if configuration loaded successfully, false otherwise
   */
  bool Load(const std::shared_ptr<const sdf::Element> &_sdf, const std::string &_pluginName);

  // Sensor parameters
  std::string topicName = "/mmwave/points";
  double updateRate = 10.0;  // Hz
  std::chrono::steady_clock::duration updatePeriod{std::chrono::milliseconds(100)};
  double horizontalFov = 1.5708;  // 90 degrees in radians
  double verticalFov = 0.5236;    // 30 degrees in radians
  double minRange = 0.05;  // meters
  double maxRange = 50.0;  // meters
  double horizontalResolution = 1.0;
  double verticalResolution = 1.0;

  // Noise parameters
  double noiseMean = 0.0;    // meters
  double noiseStdDev = 0.01; // meters
  
  // Radar cross-section parameters
  double defaultRCS = 1.0;  // m^2
  double minRCS = 0.1;      // m^2
  
  // Doppler velocity parameters
  double maxRadialVelocity = 30.0;  // m/s
  
  // Mode configuration
  bool visualize = false;
  bool usePointCloudsOnly = true;
  bool forceRaycast = false;
  bool wslCompatMode = false;
  
  // WSL compatibility parameters
  std::chrono::milliseconds worldModelUpdatePeriod{500};  // Update simplified world model every 500ms
};

}  // namespace olympus_sim

#endif  // OLYMPUS_SIM_MMWAVE_SENSOR_CONFIG_HH_
