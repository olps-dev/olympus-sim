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
  utils/PointCloudLoader.cc
  wsl_compat/wsl_compat.cc
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

# Register the plugin with Gazebo so it can be found
gz_plugin_register(
  NAME MmWaveSensorPlugin
  LIBRARY MmWaveSensorPlugin
)


# Messaging related files
set(MESSAGING_SOURCES
  messaging/MmWaveMessageHandler.cc
  messaging/OlympusMQTTPlugin.cc
)

# Sensor configuration files
set(SENSOR_CONFIG_SOURCES
  sensor_config/MmWaveSensorConfig.cc
)

# Create shared libraries for the plugins
add_library(MmWaveSensorPlugin SHARED 
  ${CORE_SOURCES}
  ${POINTCLOUD_SOURCES}
  ${RAYCAST_SOURCES}
  ${SENSOR_CONFIG_SOURCES}
  messaging/MmWaveMessageHandler.cc
)

# Create MQTT plugin library
add_library(OlympusMQTTPlugin SHARED 
  messaging/OlympusMQTTPlugin.cc
)

# Target specific include directories (if your .hh files are in an 'include' subdirectory)
# target_include_directories(MmWaveSensorPlugin PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)

# Link against Gazebo libraries
target_link_libraries(MmWaveSensorPlugin
  PRIVATE
    gz-sim8::gz-sim8
    gz-rendering8::gz-rendering8
    gz-transport13::gz-transport13
    gz-msgs10::gz-msgs10
    gz-common5::profiler
    gz-plugin2::register
    gz-math7::gz-math7
)

target_link_libraries(OlympusMQTTPlugin
  PRIVATE
    gz-sim8::gz-sim8
    gz-transport13::gz-transport13
    gz-msgs10::gz-msgs10
    gz-common5::profiler
    gz-plugin2::register
    gz-math7::gz-math7
    ${MOSQUITTO_LIBRARY}
)

# Installation
# Installs the plugin libraries to a standard location
install(TARGETS MmWaveSensorPlugin OlympusMQTTPlugin
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/gz-sim-8/plugins
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# This ensures that Gazebo can find the plugin.
# You might need to set GZ_SIM_SYSTEM_PLUGIN_PATH environment variable
# to include the build or install directory if it's not a standard path.
# Example: export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:$HOME/olympus-sim/build/sim/gazebo/plugins
# Or after install: export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:/usr/local/lib/gz-sim-8/plugins (if installed to /usr/local)

# Print install path for convenience
message(STATUS "Plugin will be installed to: ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/gz-sim-8/plugins")
