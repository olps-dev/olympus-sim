#include "sensor_config/MmWaveSensorConfig.hh"

using namespace olympus_sim;

bool MmWaveSensorConfig::Load(const std::shared_ptr<const sdf::Element> &_sdf, const std::string &_pluginName)
{
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
    this->horizontalResolution = _sdf->Get<double>("horizontal_resolution");
  }
  if (_sdf->HasElement("vertical_resolution"))
  {
    this->verticalResolution = _sdf->Get<double>("vertical_resolution");
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
  if (_sdf->HasElement("use_point_clouds_only"))
  {
    this->usePointCloudsOnly = _sdf->Get<bool>("use_point_clouds_only");
  }
  if (_sdf->HasElement("force_raycast"))
  {
    this->forceRaycast = _sdf->Get<bool>("force_raycast");
  }

  // Log the loaded parameters
  gzmsg << "[" << _pluginName << "] Parameters loaded: \n"
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
        << "  Visualize: " << (this->visualize ? "true" : "false") << "\n"
        << "  Use Point Clouds Only: " << (this->usePointCloudsOnly ? "true" : "false") << "\n"
        << "  Force Raycast: " << (this->forceRaycast ? "true" : "false") << "\n";

  return true;
}
