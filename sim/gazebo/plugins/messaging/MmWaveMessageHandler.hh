#ifndef OLYMPUS_SIM_MMWAVE_MESSAGE_HANDLER_HH_
#define OLYMPUS_SIM_MMWAVE_MESSAGE_HANDLER_HH_

#include <gz/msgs/pointcloud_packed.pb.h>
#include <string>

namespace olympus_sim
{

/**
 * @brief Class to handle point cloud message formatting for MmWave sensor
 * 
 * This class separates message creation and formatting from the main plugin logic
 */
class MmWaveMessageHandler
{
public:
  /**
   * @brief Create a new point cloud message with appropriate fields
   * @param frameId Frame ID for the message header
   * @param timestamp Simulation time for the message header
   * @return Initialized point cloud message
   */
  static gz::msgs::PointCloudPacked CreatePointCloudMessage(
      const std::int64_t &timestamp,
      const std::string &frameId);
      
  /**
   * @brief Add a test point to a point cloud message (for debugging)
   * @param pointCloudMsg The message to add the test point to
   */
  static void AddTestPoint(gz::msgs::PointCloudPacked &pointCloudMsg);
};

}  // namespace olympus_sim

#endif  // OLYMPUS_SIM_MMWAVE_MESSAGE_HANDLER_HH_
