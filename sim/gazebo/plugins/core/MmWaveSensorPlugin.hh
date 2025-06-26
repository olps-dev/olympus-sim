#ifndef OLYMPUS_SIM_MMWAVE_SENSOR_PLUGIN_HH_
#define OLYMPUS_SIM_MMWAVE_SENSOR_PLUGIN_HH_

#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/transport/Node.hh>
#include <chrono>
#include <map>
#include <random>

// Include our modular components
#include "pointcloud/MmWavePointCloudLoader.hh"
#include "sensor_config/MmWaveSensorConfig.hh"
#include "raycast/MmWaveSensorScene.hh"
#include "core/MmWaveSensorWslCompat.hh"
#include "messaging/MmWaveMessageHandler.hh"
#include "raycast/MmWaveSensorRay.hh"

namespace olympus_sim
{
  class MmWaveSensorPlugin : public gz::sim::System,
                             public gz::sim::ISystemConfigure,
                             public gz::sim::ISystemPostUpdate
  {
    public: 
      /// Constructor
      MmWaveSensorPlugin();
      
      /// Destructor
      ~MmWaveSensorPlugin() override = default;

      /// Configure the plugin
      void Configure(const gz::sim::Entity &_entity,
                     const std::shared_ptr<const sdf::Element> &_sdf,
                     gz::sim::EntityComponentManager &_ecm,
                     gz::sim::EventManager &_eventMgr) override;

      /// Update the plugin
      void PostUpdate(const gz::sim::UpdateInfo &_info,
                      const gz::sim::EntityComponentManager &_ecm) override;

    private:
      /// Initialize the point cloud loader
      bool InitializePointCloudLoader();
      
      /// Gazebo Sim entity for this plugin
      gz::sim::Entity entity;

      /// Gazebo transport node for communication
      gz::transport::Node node;

      /// Gazebo transport publisher for point cloud data
      gz::transport::Node::Publisher pointCloudPublisher;

      /// Sensor configuration
      MmWaveSensorConfig config;
      
      /// Scene manager
      std::unique_ptr<MmWaveSensorScene> sceneManager;
  
      /// Point cloud loader for pre-computed point clouds
      std::unique_ptr<MmWavePointCloudLoader> pointCloudLoader;
      
      /// Last update time for rate limiting
      std::chrono::steady_clock::duration lastUpdateTime{std::chrono::seconds(0)};
      
      /// WSL compatibility mode flag
      bool wslCompatMode{false};
      
      /// Topic to publish point cloud data on
      std::string topicName;
      
      /// Visualize flag for debugging
      bool visualize{false};
      
      /// Message handler for point cloud formatting
      std::unique_ptr<MmWaveMessageHandler> messageHandler;
      
      /// Ray sensor for ray casting
      std::unique_ptr<MmWaveSensorRay> raySensor;
      
      /// Random number generator for noise
      std::normal_distribution<double> gaussianNoise;
      
      /// Random engine for noise generation
      std::mt19937 randomEngine;
      
      /// Warning shown flag for WSL compatibility
      bool wslWarningShown{false};
      
      /// Last time the world model was updated
      std::chrono::time_point<std::chrono::steady_clock> lastWorldModelUpdate;
      
      /// Simplified world model for WSL compatibility
      std::map<gz::sim::Entity, std::pair<gz::math::Pose3d, gz::math::Vector3d>> simplifiedWorldModel;
  };
}

#endif // OLYMPUS_SIM_MMWAVE_SENSOR_PLUGIN_HH_
