#ifndef OLYMPUS_SIM_MMWAVE_SENSOR_PLUGIN_HH_
#define OLYMPUS_SIM_MMWAVE_SENSOR_PLUGIN_HH_

#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/pointcloud_packed.pb.h> // For publishing point cloud data

namespace olympus_sim
{
  class MmWaveSensorPlugin : public gz::sim::System,
                             public gz::sim::ISystemConfigure,
                             public gz::sim::ISystemPostUpdate
  {
    public: MmWaveSensorPlugin();
    public: ~MmWaveSensorPlugin() override = default;

    // Documentation inherited
    public: void Configure(const gz::sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PostUpdate(const gz::sim::UpdateInfo &_info,
                            const gz::sim::EntityComponentManager &_ecm) override;

    private:
      // Gazebo Sim entity for this plugin
      gz::sim::Entity entity;

      // Gazebo transport node for communication
      gz::transport::Node node;

      // Gazebo transport publisher for point cloud data
      gz::transport::Node::Publisher pointCloudPublisher;

      // SDF element for configuration
      std::shared_ptr<const sdf::Element> sdfConfig;

      // Topic to publish point cloud data on
      std::string topicName = "/mmwave_sensor/points_gz"; // Default topic

      // Sensor parameters (to be loaded from SDF)
      double updateRate = 10.0; // Hz
      double horizontalFov = 1.5708; // radians (90 degrees)
      double verticalFov = 0.5236; // radians (30 degrees)
      double minRange = 0.1; // meters
      double maxRange = 50.0; // meters
      int horizontalResolution = 128;
      int verticalResolution = 32;
      // Add more parameters like noise models, etc.

      // Last update time
      std::chrono::steady_clock::duration lastUpdateTime{0};
      std::chrono::steady_clock::duration updatePeriod{0};
  };
}

#endif // OLYMPUS_SIM_MMWAVE_SENSOR_PLUGIN_HH_
