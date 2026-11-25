# MoveIt Pro Experimental Behaviors

**This repository contains independent ROS 2 Behavior packages that can be copied into a MoveIt Pro user workspace.**

Each `src/*_behaviors/` directory is a complete package. The repository root is a colcon workspace, not a ROS
package.
The Behaviors are experimental and are not part of the MoveIt Pro product. Target: MoveIt Pro 10.x, ROS 2 Jazzy.

## Copy only what you need

For example, to use only YAML Behaviors:

```bash
cp -R /path/to/moveit_pro_experimental_behaviors/src/yaml_behaviors /path/to/user_workspace/src/
cd /path/to/user_workspace
moveit_pro build user_workspace
```

Restart MoveIt Pro after the build. Each package's `behavior_plugin.yaml` adds its own loader automatically.
You do not need to edit a parent manifest, use `add_subdirectory`, call a registration function or copy other
Behavior packages. Keep the selected package's license notices.

To use all packages, clone this repository under `src/`; colcon discovers the package manifests below it.

## Packages

| Package | Behavior IDs | Purpose |
| --- | ---: | --- |
| [blackboard_behaviors](src/blackboard_behaviors/README.md) | 2 | Read and write blackboard entries whose key is computed at run time. |
| [control_interface_behaviors](src/control_interface_behaviors/README.md) | 6 | Build, read and publish ros2_control interface values, such as GPIO commands and states. |
| [robot_model_behaviors](src/robot_model_behaviors/README.md) | 2 | Query the loaded robot model: joint limits and forward kinematics of a trajectory. |
| [ros_messaging_behaviors](src/ros_messaging_behaviors/README.md) | 1 | Wait for a ROS message on a topic and copy it to the blackboard. |
| [yaml_behaviors](src/yaml_behaviors/README.md) | 2 | Change values and lists in a YAML file on disk, safely and atomically. |

Find IDs and ports in the [combined offline behavior index](docs/behavior_index/index.html), or in each
package's own `docs/behavior_index/index.html`. Each package README explains its Behaviors and requirements.

## Migration from the bulk package

The old `experimental_behaviors` package and its shared loader are removed. Remove
`experimental_behaviors::ExperimentalBehaviorsLoader` from robot configs. Copy only the packages you need.
Every existing Behavior ID and port is retained, so Objectives do not need changes. C++ namespaces, header
paths and loader names are now package-specific; update external C++ includes if you use them. Do not keep
another copy of a loader that registers the same IDs.

The sources are ported to the MoveIt Pro 10.x C++ API. MoveIt Pro 10.0 removed the `RobotJointState`
message, so Behaviors that took it now take `moveit_msgs/msg/RobotState`, as the built-in Behaviors do.
Their IDs and port names are unchanged. See the 10.0 user workspace migration guide.

## Development

Build and test in the [CI MoveIt Pro image](.github/workflows/CI.yaml), never on the Mac host:

```bash
source /opt/ros/jazzy/setup.bash
source /opt/overlay_ws/install/setup.bash
colcon build --cmake-args -DBUILD_TESTING=ON
colcon test --executor sequential
colcon test-result --verbose
```

Also build each changed package by itself in a clean workspace to check its dependencies. Root layout checks
need only Python: `python3 test/test_package_isolation.py -v`. Run `pre-commit run --all-files` before committing.

When adding a Behavior, work entirely inside the relevant package, keep its ID unique, and update its tests
and port index. Then regenerate the [combined index](docs/behavior_index/README.md). Keep detailed comments
in that package's `docs/AGENT_COMMENTS.md`. Do not add shared source or CMake helpers at the repository root.
