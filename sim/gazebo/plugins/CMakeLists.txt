cmake_minimum_required(VERSION 3.10.2)
project(olympus_gazebo_plugins)

# Find Gazebo Sim (Harmonic is version 8) and its components
find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-common5 REQUIRED COMPONENTS profiler)
find_package(gz-sim8 REQUIRED)
find_package(gz-rendering8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-math7 REQUIRED)

# Add more find_package for gz-rendering if direct raycasting API is used from there

# Include directories
include_directories(
  include
  ${gz-sim8_INCLUDE_DIRS}
  ${gz-plugin2_INCLUDE_DIRS}
  ${gz-transport13_INCLUDE_DIRS}
  ${gz-msgs10_INCLUDE_DIRS}
  ${gz-math7_INCLUDE_DIRS}
)

# Create a shared library for the plugin
add_library(MmWaveSensorPlugin SHARED MmWaveSensorPlugin.cc)

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

# Installation
# Installs the plugin library to a standard location
install(TARGETS MmWaveSensorPlugin
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
