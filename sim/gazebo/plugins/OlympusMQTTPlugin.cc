#include "OlympusMQTTPlugin.hh"

#include <iostream>
#include <mosquitto.h>
#include <thread>
#include <chrono>

#include <gz/common/Console.hh>
#include <gz/common/Profiler.hh>
#include <gz/math/Color.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/rendering/Material.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Text.hh>
#include <gz/rendering/Visual.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/sim/rendering/MarkerManager.hh>
#include <gz/transport/Node.hh>
#include <sdf/sdf.hh>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace olympus
{
  // Internal implementation
  class OlympusMQTTPluginPrivate
  {
    /// \brief MQTT broker host
    public: std::string brokerHost = "localhost";

    /// \brief MQTT broker port
    public: int brokerPort = 1883;

    /// \brief Unique client ID
    public: std::string clientId;

    /// \brief Topic prefix
    public: std::string topicPrefix = "olympus";

    /// \brief Entity ID
    public: gz::sim::Entity entity;

    /// \brief Entity name
    public: std::string entityName;

    /// \brief Mosquitto client instance
    public: struct mosquitto *mosq = nullptr;

    /// \brief MQTT background thread
    public: std::thread mqttThread;

    /// \brief Flag to stop MQTT thread
    public: bool stopThread = false;

    /// \brief Message queue from MQTT thread to main thread
    public: std::vector<std::pair<std::string, std::string>> messageQueue;

    /// \brief Mutex for message queue
    public: std::mutex queueMutex;

    /// \brief Subscription topics
    public: std::vector<std::string> topics;

    // Visualization is disabled for now as the MarkerManager API has changed in Gazebo Sim v8
    // public: std::shared_ptr<gz::sim::v8::MarkerManager> markerManager;

    /// \brief Visual name prefix for markers
    public: std::string visualNamePrefix;

    /// \brief Transport node
    public: gz::transport::Node node;

    /// \brief Type of visualization (text, color, etc.)
    public: std::string visualType = "text";

    /// \brief Parent link name for visualization
    public: std::string parentLink;

    /// \brief Position offset for visualization
    public: gz::math::Vector3d position{0, 0, 1.0};

    /// \brief Data field to visualize from MQTT JSON message
    public: std::string dataField = "value";

    /// \brief The last updated value for visualization
    public: std::string lastValue;

    /// \brief Callback for MQTT connect event
    public: static void OnConnect(struct mosquitto *mosq, void *obj, int rc)
    {
      if (rc == 0)
      {
        std::cout << "MQTT Connected successfully" << std::endl;
        OlympusMQTTPluginPrivate *priv = static_cast<OlympusMQTTPluginPrivate*>(obj);
        
        // Subscribe to topics
        for (const auto &topic : priv->topics)
        {
          std::string fullTopic = priv->topicPrefix + "/" + topic;
          mosquitto_subscribe(mosq, nullptr, fullTopic.c_str(), 0);
          std::cout << "Subscribed to topic: " << fullTopic << std::endl;
        }
      }
      else
      {
        std::cerr << "MQTT Connect failed with code " << rc << std::endl;
      }
    }

    /// \brief Callback for MQTT message event
    public: static void OnMessage(struct mosquitto *mosq, void *obj, 
                                const struct mosquitto_message *msg)
    {
      OlympusMQTTPluginPrivate *priv = static_cast<OlympusMQTTPluginPrivate*>(obj);
      if (msg->payloadlen > 0)
      {
        std::string topic(msg->topic);
        std::string payload(static_cast<char*>(msg->payload), msg->payloadlen);
        
        // Add message to queue for processing in main thread
        std::lock_guard<std::mutex> lock(priv->queueMutex);
        priv->messageQueue.push_back({topic, payload});
      }
    }

    /// \brief MQTT thread function
    public: void MQTTThreadFunc()
    {
      while (!this->stopThread)
      {
        // Process MQTT messages
        mosquitto_loop(this->mosq, 100, 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }

    /// \brief Process queued messages
    public: void ProcessMessages()
    {
      std::vector<std::pair<std::string, std::string>> messages;
      {
        std::lock_guard<std::mutex> lock(this->queueMutex);
        if (this->messageQueue.empty())
          return;
        
        messages.swap(this->messageQueue);
      }

      for (const auto &msg : messages)
      {
        try
        {
          // Parse JSON payload
          json j = json::parse(msg.second);
          
          // Extract value based on dataField
          if (j.contains(this->dataField))
          {
            auto value = j[this->dataField];
            std::string valueStr;
            
            if (value.is_string())
              valueStr = value.get<std::string>();
            else if (value.is_number())
              valueStr = std::to_string(value.get<double>());
            else if (value.is_boolean())
              valueStr = value.get<bool>() ? "true" : "false";
            else
              valueStr = value.dump();
            
            this->lastValue = valueStr;
            
            // Update visualization based on type
            if (this->visualType == "text")
            {
              this->UpdateTextVisualization(valueStr);
            }
            else if (this->visualType == "color")
            {
              this->UpdateColorVisualization(valueStr);
            }
            // Add more visualization types as needed
          }
        }
        catch (const std::exception &e)
        {
          std::cerr << "Error processing MQTT message: " << e.what() << std::endl;
        }
      }
    }

    /// \brief Update text visualization
    public: void UpdateTextVisualization(const std::string &text)
    {
      // Visualization disabled in this version
      // The MarkerManager API has changed in Gazebo Sim v8
      std::cout << "Text Visualization [" << this->visualNamePrefix << "]: " << text << std::endl;
    }

    /// \brief Update color visualization
    public: void UpdateColorVisualization(const std::string &value)
    {
      // Visualization disabled in this version
      // The MarkerManager API has changed in Gazebo Sim v8
      std::cout << "Color Visualization [" << this->visualNamePrefix << "]: " << value << std::endl;
    }
  };

  //////////////////////////////////////////////////
  OlympusMQTTPlugin::OlympusMQTTPlugin()
      : dataPtr(std::make_unique<OlympusMQTTPluginPrivate>())
  {
    // Initialize mosquitto library once
    mosquitto_lib_init();
  }

  //////////////////////////////////////////////////
  OlympusMQTTPlugin::~OlympusMQTTPlugin()
  {
    // Stop MQTT thread
    if (this->dataPtr->mosq)
    {
      this->dataPtr->stopThread = true;
      if (this->dataPtr->mqttThread.joinable())
        this->dataPtr->mqttThread.join();
      
      mosquitto_disconnect(this->dataPtr->mosq);
      mosquitto_destroy(this->dataPtr->mosq);
      this->dataPtr->mosq = nullptr;
    }
    
    // Cleanup mosquitto library
    mosquitto_lib_cleanup();
  }

  //////////////////////////////////////////////////
  void OlympusMQTTPlugin::Configure(const gz::sim::Entity &_entity,
                             const std::shared_ptr<const sdf::Element> &_sdf,
                             gz::sim::EntityComponentManager &_ecm,
                             gz::sim::EventManager &)
  {
    this->dataPtr->entity = _entity;
    
    // Get entity name
    if (_ecm.EntityHasComponentType(_entity, gz::sim::components::Name::typeId))
    {
      this->dataPtr->entityName =
          _ecm.Component<gz::sim::components::Name>(_entity)->Data();
    }
    
    // Generate a unique client ID if not specified
    this->dataPtr->clientId = "gazebo_" + this->dataPtr->entityName + "_" + 
                             std::to_string(static_cast<int>(std::chrono::system_clock::now()
                                           .time_since_epoch().count() % 10000));
    
    // Parse SDF parameters
    if (_sdf->HasElement("broker_host"))
      this->dataPtr->brokerHost = _sdf->Get<std::string>("broker_host");
    
    if (_sdf->HasElement("broker_port"))
      this->dataPtr->brokerPort = _sdf->Get<int>("broker_port");
    
    if (_sdf->HasElement("client_id"))
      this->dataPtr->clientId = _sdf->Get<std::string>("client_id");
    
    if (_sdf->HasElement("topic_prefix"))
      this->dataPtr->topicPrefix = _sdf->Get<std::string>("topic_prefix");
    
    if (_sdf->HasElement("visual_type"))
      this->dataPtr->visualType = _sdf->Get<std::string>("visual_type");
    
    if (_sdf->HasElement("parent_link"))
      this->dataPtr->parentLink = _sdf->Get<std::string>("parent_link");
    
    if (_sdf->HasElement("position"))
      this->dataPtr->position = _sdf->Get<gz::math::Vector3d>("position");
    
    if (_sdf->HasElement("data_field"))
      this->dataPtr->dataField = _sdf->Get<std::string>("data_field");
    
    // Parse topics to subscribe
    if (_sdf->HasElement("topic"))
    {
      // Clone the const SDF element to get a non-const version to work with
      auto sdfCopy = std::const_pointer_cast<sdf::Element>(_sdf->Clone());
      auto topicElem = sdfCopy->GetElement("topic");
      while (topicElem)
      {
        this->dataPtr->topics.push_back(topicElem->Get<std::string>());
        topicElem = topicElem->GetNextElement("topic");
      }
    }
    
    // Create visual name prefix
    this->dataPtr->visualNamePrefix = "mqtt_" + this->dataPtr->entityName;
    
    // Create MQTT client
    this->dataPtr->mosq = mosquitto_new(
      this->dataPtr->clientId.c_str(), true, this->dataPtr.get());
    
    if (!this->dataPtr->mosq)
    {
      std::cerr << "Error: Out of memory creating MQTT client" << std::endl;
      return;
    }
    
    // Set callbacks
    mosquitto_connect_callback_set(
      this->dataPtr->mosq, OlympusMQTTPluginPrivate::OnConnect);
    mosquitto_message_callback_set(
      this->dataPtr->mosq, OlympusMQTTPluginPrivate::OnMessage);
    
    // Connect to broker
    int rc = mosquitto_connect(
      this->dataPtr->mosq, 
      this->dataPtr->brokerHost.c_str(), 
      this->dataPtr->brokerPort, 
      60);  // keepalive in seconds
    
    if (rc != MOSQ_ERR_SUCCESS)
    {
      std::cerr << "Error connecting to MQTT broker: " << 
        mosquitto_strerror(rc) << std::endl;
      return;
    }
    
    // Start MQTT processing thread
    this->dataPtr->mqttThread = std::thread(
      &OlympusMQTTPluginPrivate::MQTTThreadFunc, this->dataPtr.get());
    
    // Visualization with MarkerManager is disabled in this version
    // The MarkerManager API has changed in Gazebo Sim v8
    // this->dataPtr->markerManager =
    //    std::make_shared<gz::sim::v8::MarkerManager>();
  }

  //////////////////////////////////////////////////
  void OlympusMQTTPlugin::PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm)
  {
    GZ_PROFILE("OlympusMQTTPlugin::PreUpdate");
    
    // Skip if paused
    if (_info.paused)
      return;
    
    // Process any received MQTT messages
    this->dataPtr->ProcessMessages();
  }
}

// Register plugin
GZ_ADD_PLUGIN(
    olympus::OlympusMQTTPlugin,
    gz::sim::System,
    olympus::OlympusMQTTPlugin::ISystemConfigure,
    olympus::OlympusMQTTPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(olympus::OlympusMQTTPlugin,
                  "olympus::OlympusMQTTPlugin")
