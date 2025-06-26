#include "messaging/MmWaveMessageHandler.hh"
#include <gz/common/Console.hh>
#include <gz/msgs/time.pb.h>
#include <cstring>
#include <chrono>

using namespace olympus_sim;

gz::msgs::PointCloudPacked MmWaveMessageHandler::CreatePointCloudMessage(
    const std::int64_t &timestamp,
    const std::string &frameId)
{
  gz::msgs::PointCloudPacked pointCloudMsg;
  
  // Set header with timestamp
  // Convert int64_t timestamp (nanoseconds) to seconds and nanoseconds
  std::int64_t seconds = timestamp / 1000000000;
  std::int64_t nanoseconds = timestamp % 1000000000;
  pointCloudMsg.mutable_header()->mutable_stamp()->set_sec(seconds);
  pointCloudMsg.mutable_header()->mutable_stamp()->set_nsec(nanoseconds);
  
  // Add frame ID (sensor link name)
  if (!frameId.empty())
  {
    pointCloudMsg.mutable_header()->add_data()->set_key("frame_id");
    pointCloudMsg.mutable_header()->mutable_data(0)->add_value(frameId);
  }

  // Define fields for the point cloud
  // x, y, z coordinates
  auto xField = pointCloudMsg.add_field();
  xField->set_name("x");
  xField->set_offset(0);
  xField->set_datatype(gz::msgs::PointCloudPacked::Field::FLOAT32);
  xField->set_count(1);

  auto yField = pointCloudMsg.add_field();
  yField->set_name("y");
  yField->set_offset(4); // 4 bytes after x
  yField->set_datatype(gz::msgs::PointCloudPacked::Field::FLOAT32);
  yField->set_count(1);

  auto zField = pointCloudMsg.add_field();
  zField->set_name("z");
  zField->set_offset(8); // 4 bytes after y
  zField->set_datatype(gz::msgs::PointCloudPacked::Field::FLOAT32);
  zField->set_count(1);

  // Add velocity field (for Doppler)
  auto velField = pointCloudMsg.add_field();
  velField->set_name("velocity");
  velField->set_offset(12); // 4 bytes after z
  velField->set_datatype(gz::msgs::PointCloudPacked::Field::FLOAT32);
  velField->set_count(1);

  // Add RCS field
  auto rcsField = pointCloudMsg.add_field();
  rcsField->set_name("rcs");
  rcsField->set_offset(16); // 4 bytes after velocity
  rcsField->set_datatype(gz::msgs::PointCloudPacked::Field::FLOAT32);
  rcsField->set_count(1);

  // Add intensity field (signal strength)
  auto intensityField = pointCloudMsg.add_field();
  intensityField->set_name("intensity");
  intensityField->set_offset(20); // 4 bytes after rcs
  intensityField->set_datatype(gz::msgs::PointCloudPacked::Field::FLOAT32);
  intensityField->set_count(1);

  // Set point step (total bytes per point)
  const int pointStep = 24; // 6 fields * 4 bytes
  pointCloudMsg.set_point_step(pointStep);
  pointCloudMsg.set_is_dense(true);
  
  return pointCloudMsg;
}

void MmWaveMessageHandler::AddTestPoint(gz::msgs::PointCloudPacked &pointCloudMsg)
{
  gzmsg << "[MmWaveMessageHandler] Adding test point to empty point cloud" << std::endl;
  
  // Clear any existing fields and add all required ones
  pointCloudMsg.clear_field();
  
  // Add x field
  auto field = pointCloudMsg.add_field();
  field->set_name("x");
  field->set_offset(0);
  field->set_datatype(gz::msgs::PointCloudPacked::Field::FLOAT32);
  field->set_count(1);
  
  // Add y field
  field = pointCloudMsg.add_field();
  field->set_name("y");
  field->set_offset(4);
  field->set_datatype(gz::msgs::PointCloudPacked::Field::FLOAT32);
  field->set_count(1);
  
  // Add z field
  field = pointCloudMsg.add_field();
  field->set_name("z");
  field->set_offset(8);
  field->set_datatype(gz::msgs::PointCloudPacked::Field::FLOAT32);
  field->set_count(1);
  
  // Point at 5 meters in front of the sensor
  float x = 5.0f;
  float y = 0.0f;
  float z = 0.0f;
  
  pointCloudMsg.set_point_step(12);  // 3 fields * 4 bytes (float32)
  pointCloudMsg.set_height(1);
  pointCloudMsg.set_width(1);
  pointCloudMsg.set_row_step(12);
  pointCloudMsg.set_is_dense(true);
  
  pointCloudMsg.mutable_data()->resize(12);
  std::memcpy(&pointCloudMsg.mutable_data()->at(0), &x, 4);
  std::memcpy(&pointCloudMsg.mutable_data()->at(4), &y, 4);
  std::memcpy(&pointCloudMsg.mutable_data()->at(8), &z, 4);
  
  gzmsg << "[MmWaveMessageHandler] Added test point at (" 
        << x << ", " << y << ", " << z << ")" << std::endl;
}
