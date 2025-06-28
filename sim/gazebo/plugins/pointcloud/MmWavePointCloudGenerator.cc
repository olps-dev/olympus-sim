#include "MmWavePointCloudGenerator.hh"

#include <gz/math/Rand.hh>


namespace olympus_sim
{


void MmWavePointCloudGenerator::InitializeDefaultSensorParams(
    std::shared_ptr<MmWavePointCloud> pointCloud) 
{
  // Set default sensor parameters
  pointCloud->horizontalFov = 1.5708; // 90 degrees
  pointCloud->verticalFov = 0.5236;   // 30 degrees
  pointCloud->horizontalResolution = 32;
  pointCloud->verticalResolution = 16;
  pointCloud->sensorPose = gz::math::Pose3d(0, 0, 0, 0, 0, 0); // Origin
}


void MmWavePointCloudGenerator::AddNoiseToPoints(
    std::shared_ptr<MmWavePointCloud> pointCloud, double rangeNoise) 
{
  // Add noise to all points
  for (auto& point : pointCloud->points) {
    // Add random range noise along the ray direction
    double distance = point.position.Length();
    if (distance > 0) {
      gz::math::Vector3d rayDir = point.position.Normalized();
      double noise = gz::math::Rand::DblNormal(0.0, rangeNoise);
      point.position += rayDir * noise;
    }
  }
}

} // namespace olympus_sim
