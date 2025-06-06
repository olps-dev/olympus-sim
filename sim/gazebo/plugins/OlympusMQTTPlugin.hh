#ifndef OLYMPUS_MQTT_PLUGIN_HH_
#define OLYMPUS_MQTT_PLUGIN_HH_

#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <unordered_map>

#include <gz/sim/System.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Text.hh>
#include <gz/rendering/Visual.hh>
#include <gz/transport/Node.hh>

namespace olympus
{
  // Forward declaration of private data
  class OlympusMQTTPluginPrivate;

  /// \brief A plugin that connects to an MQTT broker and updates
  /// Gazebo visualizations based on received messages
  class OlympusMQTTPlugin
      : public gz::sim::System,
        public gz::sim::ISystemConfigure,
        public gz::sim::ISystemPreUpdate
  {
    /// \brief Constructor
    public: OlympusMQTTPlugin();

    /// \brief Destructor
    public: ~OlympusMQTTPlugin() override;

    // Documentation inherited
    public: void Configure(const gz::sim::Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         gz::sim::EntityComponentManager &_ecm,
                         gz::sim::EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PreUpdate(
                const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;

    /// \brief Private data pointer
    private: std::unique_ptr<OlympusMQTTPluginPrivate> dataPtr;
  };
}

#endif  // OLYMPUS_MQTT_PLUGIN_HH_
