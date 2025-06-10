#ifndef OLYMPUS_SIM_MMWAVE_SENSOR_PLUGIN_HH_
#define OLYMPUS_SIM_MMWAVE_SENSOR_HH_

#include <gz/sim/System.hh>
#include <gz/math/Rand.hh> // For NormalRealDist and random number generation
#include <gz/transport/Node.hh>
#include <gz/msgs/pointcloud_packed.pb.h> // For publishing point cloud data
#include <gz/rendering/Scene.hh>
#include <gz/math/Angle.hh>
#include <gz/math/Vector3.hh>
#include <map> // For storing simplified world model
#include "MmWaveSensorWslCompat.hh" // For WSL compatibility mode functions

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
      // Ray casting helper method
      void CastRays(const gz::math::Pose3d &_sensorPose,
                    const gz::sim::EntityComponentManager &_ecm, 
                    gz::msgs::PointCloudPacked &_pointCloudMsg);
                    
      // Get the rendering scene
      gz::rendering::ScenePtr Scene() const;
      
      // Gazebo Sim entity for this plugin
      gz::sim::Entity entity;

      // Gazebo transport node for communication
      gz::transport::Node node;

      // Gazebo transport publisher for point cloud data
      gz::transport::Node::Publisher pointCloudPublisher;

      // SDF element for configuration
      std::shared_ptr<const sdf::Element> sdfConfig;
      
      // Sensor parameters
      std::string topicName = "/mmwave/points";
      double updateRate = 10.0;  // Hz
      std::chrono::steady_clock::duration updatePeriod{std::chrono::milliseconds(100)};
      double horizontalFov = 1.5708;  // 90 degrees in radians
      double verticalFov = 0.5236;    // 30 degrees in radians
      double minRange = 0.05;  // meters
      double maxRange = 50.0;  // meters
      int horizontalResolution = 32;
      int verticalResolution = 16;
      
      // Noise parameters
      double noiseMean = 0.0;    // meters
      double noiseStdDev = 0.01; // meters
      
      // Radar cross-section parameters
      double defaultRCS = 1.0;  // m^2
      double minRCS = 0.1;      // m^2
      
      // Doppler velocity parameters
      double maxRadialVelocity = 30.0;  // m/s
      
      // Visualization flag
      bool visualize = false;
      
      // Rendering scene for ray casting
      gz::rendering::ScenePtr scene{nullptr};
      gz::rendering::RayQueryPtr rayQuery{nullptr};
      gz::math::NormalRealDist gaussianNoise;
      
      // Last update time for rate limiting
      std::chrono::steady_clock::duration lastUpdateTime{std::chrono::seconds(0)};
      
      // WSL compatibility mode configuration
      bool wslCompatMode{false};
      std::map<gz::sim::Entity, std::pair<gz::math::Pose3d, gz::math::Vector3d>> simplifiedWorldModel;
      int sceneAcquisitionAttempts{0};
      const int maxSceneAcquisitionAttempts{5};
      bool wslWarningShown{false};
      std::chrono::time_point<std::chrono::steady_clock> lastWorldModelUpdate;
      const std::chrono::milliseconds worldModelUpdatePeriod{500}; // Update simplified world model every 500ms
  };
}

#endif // OLYMPUS_SIM_MMWAVE_SENSOR_PLUGIN_HH_
