#ifndef OLYMPUS_SIM_MMWAVE_SENSOR_PLUGIN_HH_
#define OLYMPUS_SIM_MMWAVE_SENSOR_HH_

#include <gz/sim/System.hh>
#include <gz/math/Rand.hh> // For NormalRealDist and random number generation
#include <gz/transport/Node.hh>
#include <gz/msgs/pointcloud_packed.pb.h> // For publishing point cloud data
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <gz/rendering/RayQuery.hh>
#include <gz/rendering/Scene.hh>
#include "MmWavePointCloudLoader.hh" // Add our new point cloud loader
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
                    
      // Get a rendering scene (kept for compatibility but not used for ray casting)
      gz::rendering::ScenePtr Scene() const;
      
      // Initialize the point cloud loader
      bool InitializePointCloudLoader();
      
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
      
      // Rendering scene  // The scene being rendered (kept for compatibility but not used for ray casting)
      gz::rendering::ScenePtr scene;

      // Ray query interface (kept for compatibility but not used for ray casting)
      gz::rendering::RayQueryPtr rayQuery;
  
      // Point cloud loader for pre-computed point clouds
      std::unique_ptr<MmWavePointCloudLoader> pointCloudLoader{nullptr};
      gz::math::NormalRealDist gaussianNoise;
      
      // Last update time for rate limiting
      std::chrono::steady_clock::duration lastUpdateTime{std::chrono::seconds(0)};
      
      // WSL compatibility mode flag
      bool wslCompatMode = false;

      // Use pre-computed point clouds (no ray casting)
      bool usePointCloudsOnly = true;

      // Number of scene acquisition attempts
      int sceneAcquisitionAttempts = 0;
      const int maxSceneAcquisitionAttempts = 5;
      bool wslWarningShown = false;
      std::chrono::time_point<std::chrono::steady_clock> lastWorldModelUpdate;
      const std::chrono::milliseconds worldModelUpdatePeriod{500}; // Update simplified world model every 500ms
      std::map<gz::sim::Entity, std::pair<gz::math::Pose3d, gz::math::Vector3d>> simplifiedWorldModel;
  };
}

#endif // OLYMPUS_SIM_MMWAVE_SENSOR_PLUGIN_HH_
