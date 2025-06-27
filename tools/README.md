# Tools and Utilities

This directory contains tools and utilities for the Olympus simulation.

## Files

- `launch_olympus.py` - Main unified launcher (called by `./olympus` wrapper)
- `manipulate_scene.py` - Interactive scene manipulation tool
- `verify_setup.py` - Setup verification and health check

## Launcher

The unified launcher provides a single entry point for all simulation modes:
- Handles process management and environment setup
- Supports multiple modes: `full`, `gazebo`, `sensor`, `automation`
- Provides options: `--gui`, `--rviz`, `--automation`

## Setup Verification

Check that all components are properly organized and accessible:

```bash
python3 tools/verify_setup.py
```

This verifies:
- Directory structure is correct
- All core components are present
- Documentation files exist
- ROS2 and Gazebo files are accessible

## Scene Manipulation

Interactive tool for testing sensor responses by moving objects in the simulation:

```bash
# Start simulation first
./olympus full --gui

# In another terminal
python3 tools/manipulate_scene.py
```

### Commands:
- `list` - Show all entities in the scene
- `move <entity> <x> <y> <z>` - Move an object to coordinates
- `spawn <model> <x> <y> <z>` - Add new objects
- `delete <entity>` - Remove objects
- `help` - Show all commands
- `quit` - Exit the tool

Perfect for testing how the mmWave sensor responds to object movement and presence detection.
