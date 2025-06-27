# Legacy Launch Scripts

These are the original launch scripts that have been replaced by the unified launcher system.

## Migration Guide

### Old → New

| Old Command | New Command |
|-------------|-------------|
| `./run_simulation.sh` | `./olympus full` |
| `./run_simulation.sh --gui` | `./olympus full --gui` |
| `python3 run_olympus.py --gazebo` | `./olympus gazebo` |
| Manual RViz launch | `./olympus full --rviz` |

### Why the Change?

The old system had multiple entry points and scattered launch scripts:
- `run_simulation.sh` - Shell script launcher
- `run_olympus.py` - Python launcher
- Various scripts in `scripts/launch/`
- Manual ROS2 launch files

The new unified system provides:
- ✅ Single entry point (`./olympus`)
- ✅ Clear mode selection
- ✅ Proper process management
- ✅ Consistent environment setup
- ✅ Better error handling

## Legacy Scripts Still Available

If you need the old behavior, these scripts are still functional:
- `run_olympus.py` - Original Python launcher
- `run_simulation.sh` - Original shell launcher

But we recommend migrating to the new unified launcher for better reliability and features.
