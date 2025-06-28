#include "sensor_config/MmWaveSensorConfig.hh"

using namespace olympus_sim;

bool MmWaveSensorConfig::Load(const std::shared_ptr<const sdf::Element> &_sdf, const std::string &_pluginName, const std::string &_sensorName)
{
  gzmsg << "[" << _pluginName << "] Starting config load, current topic: " << this->topicName << std::endl;
  
  // First try to get topic from plugin configuration (for backward compatibility)
  if (_sdf->HasElement("topic"))
  {
    this->topicName = _sdf->Get<std::string>("topic");
    gzmsg << "[" << _pluginName << "] Found topic in plugin config: " << this->topicName << std::endl;
  }
  else
  {
    gzmsg << "[" << _pluginName << "] No topic in plugin config, trying alternative methods" << std::endl;
    
    // Try multiple approaches to find the topic
    bool topicFound = false;
    
    // Method 1: Try to get topic from parent sensor element using GetParent
    auto currentElement = _sdf;
    int traversalDepth = 0;
    while (currentElement && traversalDepth < 5)
    {
      gzmsg << "[" << _pluginName << "] Traversal depth " << traversalDepth 
            << ", element name: " << currentElement->GetName() << std::endl;
      
      // Check if current element has topic
      if (currentElement->HasElement("topic"))
      {
        this->topicName = currentElement->Get<std::string>("topic");
        gzmsg << "[" << _pluginName << "] Found topic at depth " << traversalDepth 
              << ": " << this->topicName << std::endl;
        topicFound = true;
        break;
      }
      
      // Check if we're at sensor element
      if (currentElement->GetName() == "sensor")
      {
        // Even if no topic element, check for topic attribute
        if (currentElement->HasAttribute("topic"))
        {
          this->topicName = currentElement->Get<std::string>("topic");
          gzmsg << "[" << _pluginName << "] Found topic attribute in sensor: " << this->topicName << std::endl;
          topicFound = true;
          break;
        }
        
        // Also check direct children for topic
        // Note: We need to check if element exists first before trying to get its value
        if (currentElement->HasElement("topic"))
        {
          // For const elements, we can directly get the value
          this->topicName = currentElement->Get<std::string>("topic");
          gzmsg << "[" << _pluginName << "] Found topic child in sensor: " << this->topicName << std::endl;
          topicFound = true;
          break;
        }
      }
      
      currentElement = currentElement->GetParent();
      traversalDepth++;
    }
    
    // Method 2: If parent traversal failed, try a different approach
    // Some Gazebo versions might not properly link parent elements
    if (!topicFound)
    {
      gzmsg << "[" << _pluginName << "] Parent traversal failed, trying direct sensor access" << std::endl;
      
      // In some cases, the sensor configuration might be available through
      // a different mechanism. Let's check if the plugin has access to 
      // sensor-specific configuration
      auto rayParent = _sdf->GetParent();
      if (rayParent && rayParent->GetName() == "ray")
      {
        gzmsg << "[" << _pluginName << "] Found ray parent, checking for sensor sibling" << std::endl;
        
        // Sometimes the topic is a sibling of the ray element
        auto sensorParent = rayParent->GetParent();
        if (sensorParent && sensorParent->GetName() == "sensor")
        {
          gzmsg << "[" << _pluginName << "] Found sensor parent, looking for topic element" << std::endl;
          
          // Try to find topic as a direct child
          if (sensorParent->HasElement("topic"))
          {
            this->topicName = sensorParent->Get<std::string>("topic");
            gzmsg << "[" << _pluginName << "] Found topic in sensor parent: " << this->topicName << std::endl;
            topicFound = true;
          }
        }
      }
    }
    
    if (!topicFound)
    {
      // Final fallback: if sensor name is provided and different from default, generate topic from sensor name
      if (!_sensorName.empty())
      {
        // If sensor name is "mmwave", use default topic
        // If sensor name is "mmwave2", "mmwave3", etc., generate topic accordingly
        if (_sensorName == "mmwave")
        {
          this->topicName = "/mmwave/points";
          gzmsg << "[" << _pluginName << "] Using default topic for sensor 'mmwave': " << this->topicName << std::endl;
        }
        else if (_sensorName.find("mmwave") == 0)
        {
          // Extract the sensor name and create topic
          this->topicName = "/" + _sensorName + "/points";
          gzmsg << "[" << _pluginName << "] Generated topic from sensor name '" << _sensorName << "': " << this->topicName << std::endl;
        }
        else
        {
          gzmsg << "[" << _pluginName << "] Sensor name '" << _sensorName << "' doesn't match mmwave pattern, using default: " << this->topicName << std::endl;
        }
      }
      else
      {
        gzmsg << "[" << _pluginName << "] No sensor name provided, using default: " << this->topicName << std::endl;
      }
    }
  }

  // Load parameters from SDF
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
  if (_sdf->HasElement("wsl_compat_mode"))
  {
    this->wslCompatMode = _sdf->Get<bool>("wsl_compat_mode");
    gzmsg << "[" << _pluginName << "] Found wsl_compat_mode parameter: " << (this->wslCompatMode ? "true" : "false") << std::endl;
  }
  else
  {
    gzmsg << "[" << _pluginName << "] wsl_compat_mode parameter not found in SDF, using default: " << (this->wslCompatMode ? "true" : "false") << std::endl;
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
        << "  Force Raycast: " << (this->forceRaycast ? "true" : "false") << "\n"
        << "  WSL Compat Mode: " << (this->wslCompatMode ? "true" : "false") << "\n";

  return true;
}
