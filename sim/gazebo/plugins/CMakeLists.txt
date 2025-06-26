cmake_minimum_required(VERSION 3.10.2)
project(olympus_gazebo_plugins)

# Set C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find all necessary Gazebo packages
find_package(gz-cmake3 REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-rendering8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)

# Include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
)

# Define the source files for the MmWaveSensorPlugin
set(MMWAVE_PLUGIN_SOURCES
  core/MmWaveSensorPlugin.cc
  messaging/MmWaveMessageHandler.cc
  raycast/MmWaveSensorRay.cc
  raycast/MmWaveSensorScene.cc
  pointcloud/MmWavePointCloudLoader.cc
  core/MmWaveSensorWslCompat.cc
  sensor_config/MmWaveSensorConfig.cc
)

# Create the shared library for the plugin
add_library(MmWaveSensorPlugin SHARED ${MMWAVE_PLUGIN_SOURCES})

# Link the plugin against the required Gazebo libraries
target_link_libraries(MmWaveSensorPlugin
  PRIVATE
    gz-sim8::gz-sim8
    gz-rendering8::gz-rendering8
    gz-transport13::gz-transport13
    gz-msgs10::gz-msgs10
    gz-math7::gz-math7
    gz-plugin2::gz-plugin2
)

# Create MQTT plugin library
add_library(OlympusMQTTPlugin SHARED 
  messaging/OlympusMQTTPlugin.cc
)

target_link_libraries(OlympusMQTTPlugin
  PRIVATE
    gz-sim8::gz-sim8
    gz-transport13::gz-transport13
    gz-msgs10::gz-msgs10
    gz-common5::profiler
    gz-plugin2::register
)

# Set the install directory for the plugins
set(CMAKE_INSTALL_PREFIX "/usr/local/")
message(STATUS "Plugin will be installed to: ${CMAKE_INSTALL_PREFIX}/gz-sim-8/plugins")

# Install the plugin library
install(TARGETS MmWaveSensorPlugin OlympusMQTTPlugin
  LIBRARY DESTINATION gz-sim-8/plugins
  ARCHIVE DESTINATION gz-sim-8/plugins
  RUNTIME DESTINATION gz-sim-8/plugins
)
