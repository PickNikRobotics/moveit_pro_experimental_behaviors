# ROS messaging Behaviors

**`ros_messaging_behaviors` is a standalone ROS 2 package of MoveIt Pro Behaviors.**

Wait for a ROS message on a topic and copy it to the blackboard.


## Use this package

Requires MoveIt Pro 10.x on ROS 2 Jazzy. It uses the PickNik BehaviorTree.CPP fork.

1. Copy this directory to `<user_workspace>/src/ros_messaging_behaviors/`. Keep its `LICENSE` and `NOTICE`.
2. Build with `moveit_pro build user_workspace`, then restart MoveIt Pro.
3. Use the Behavior IDs below in your Objectives.

`behavior_plugin.yaml` automatically registers `ros_messaging_behaviors::RosMessagingBehaviorsLoader` in workspace configs.
No parent package, copied CMake helpers, manual registration call or sibling package is required.
MoveIt Pro, ROS 2 and system dependencies are declared in [package.xml](package.xml).

Remove the old `experimental_behaviors::ExperimentalBehaviorsLoader` entry from your config and remove
the old bulk package if it is present. Do not load another plugin with the same Behavior IDs; BT.CPP
rejects duplicate IDs. For a setup that does not read `behavior_plugin.yaml`, use this loader entry:

```yaml
objectives:
  behavior_loader_plugins:
    ros_messaging_behaviors:
      - "ros_messaging_behaviors::RosMessagingBehaviorsLoader"
```

## Behaviors

| Behavior ID | What it does |
| ----------- | ------------ |
| `GetPoseStampedFromTopic` | Waits for a `geometry_msgs/PoseStamped` message on a topic and writes it to the blackboard. |

Every port is documented in the [offline behavior index](docs/behavior_index/index.html).

## Package layout

```text
ros_messaging_behaviors/
├── CMakeLists.txt, package.xml, README.md, LICENSE, NOTICE
├── behavior_plugin.yaml, ros_messaging_behaviors_plugin_description.xml
├── include/ros_messaging_behaviors/
├── src/                         Behaviors, helpers and this package's loader
├── test/                        plugin, port index and unit tests
└── docs/                        offline port page and AGENT_COMMENTS.md
```

## Build and test

Inside a MoveIt Pro 10.x image, from a workspace containing only this package or several packages:

```bash
source /opt/ros/jazzy/setup.bash
source /opt/overlay_ws/install/setup.bash
colcon build --packages-select ros_messaging_behaviors --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select ros_messaging_behaviors --executor sequential
colcon test-result --verbose
```

The plugin test loads this package's installed loader and creates every registered Behavior. The index
test checks that its documented IDs and ports match the code. These tests need no sibling packages.

When adding a Behavior, update this package's sources, dependencies and registration function, add tests,
then [regenerate the index](docs/behavior_index/README.md).

## Notes

- This folder has one Behavior now. Other generic topic and service Behaviors go here too.
- `GetDynamicInterfaceGroupValues` also reads a topic, but it is in `control_interface_behaviors` because it is only useful with the other interface value Behaviors.
