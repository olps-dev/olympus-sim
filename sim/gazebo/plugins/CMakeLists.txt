cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(OlympusMQTTPlugin)

# Find dependencies
find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-common5 REQUIRED COMPONENTS profiler)
find_package(gz-sim8 REQUIRED)
find_package(gz-rendering8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

# Find Mosquitto MQTT library
find_path(MOSQUITTO_INCLUDE_DIR mosquitto.h)
find_library(MOSQUITTO_LIBRARY mosquitto)
if(NOT MOSQUITTO_INCLUDE_DIR OR NOT MOSQUITTO_LIBRARY)
  message(FATAL_ERROR "Mosquitto library not found")
endif()

# Find nlohmann_json
include(FetchContent)
FetchContent_Declare(json
  URL https://github.com/nlohmann/json/releases/download/v3.11.2/json.tar.xz)
FetchContent_MakeAvailable(json)

# Add the plugin library
gz_add_plugin(OlympusMQTTPlugin
  SOURCES
    OlympusMQTTPlugin.cc
  INCLUDE_DIRS
    ${MOSQUITTO_INCLUDE_DIR}
  PRIVATE_LINK_LIBS
    ${MOSQUITTO_LIBRARY}
    nlohmann_json::nlohmann_json
  PUBLIC_LINK_LIBS
    gz-sim8::gz-sim8
    gz-rendering8::gz-rendering8
    gz-transport13::gz-transport13
    gz-msgs10::gz-msgs10
    gz-common5::profiler
)

# Install the plugin
install(TARGETS OlympusMQTTPlugin
  DESTINATION lib/gz-sim-8/plugins)

# Install the header (optional)
install(FILES OlympusMQTTPlugin.hh
  DESTINATION include/gz/sim/plugins)
