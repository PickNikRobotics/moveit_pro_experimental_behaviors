# Control interface Behaviors

**`control_interface_behaviors` is a standalone ROS 2 package of MoveIt Pro Behaviors.**

Build, read and publish ros2_control interface values (`control_msgs/InterfaceValue` and `control_msgs/DynamicInterfaceGroupValues`). Use them, for example, to send GPIO commands to a controller and to read GPIO states back.


## Use this package

Requires MoveIt Pro 10.x on ROS 2 Jazzy. It uses the PickNik BehaviorTree.CPP fork.

1. Copy this directory to `<user_workspace>/src/control_interface_behaviors/`. Keep its `LICENSE` and `NOTICE`.
2. Build with `moveit_pro build user_workspace`, then restart MoveIt Pro.
3. Use the Behavior IDs below in your Objectives.

`behavior_plugin.yaml` automatically registers `control_interface_behaviors::ControlInterfaceBehaviorsLoader` in workspace configs.
No parent package, copied CMake helpers, manual registration call or sibling package is required.
MoveIt Pro, ROS 2 and system dependencies are declared in [package.xml](package.xml).

Remove the old `experimental_behaviors::ExperimentalBehaviorsLoader` entry from your config and remove
the old bulk package if it is present. Do not load another plugin with the same Behavior IDs; BT.CPP
rejects duplicate IDs. For a setup that does not read `behavior_plugin.yaml`, use this loader entry:

```yaml
objectives:
  behavior_loader_plugins:
    control_interface_behaviors:
      - "control_interface_behaviors::ControlInterfaceBehaviorsLoader"
```

## Behaviors

| Behavior ID | What it does |
| ----------- | ------------ |
| `CreateInterfaceValue` | Makes one `InterfaceValue` message from a list of interface names and a list of values. |
| `CreateDynamicInterfaceGroupValues` | Makes one `DynamicInterfaceGroupValues` message from a list of group names and a list of `InterfaceValue` messages. |
| `PublishDynamicInterfaceGroupValues` | Publishes a `DynamicInterfaceGroupValues` message on a topic. |
| `GetDynamicInterfaceGroupValues` | Waits for a `DynamicInterfaceGroupValues` message on a topic and writes it to the blackboard. |
| `GetInterfaceValueFromGroup` | Finds one interface group by name in a `DynamicInterfaceGroupValues` message and outputs its `InterfaceValue`. |
| `AccessInterfaceValue` | Finds one interface by name in an `InterfaceValue` message and outputs its value as a `double`. |

Every port is documented in the [offline behavior index](docs/behavior_index/index.html).

## Package layout

```text
control_interface_behaviors/
├── CMakeLists.txt, package.xml, README.md, LICENSE, NOTICE
├── behavior_plugin.yaml, control_interface_behaviors_plugin_description.xml
├── include/control_interface_behaviors/
├── src/                         Behaviors, helpers and this package's loader
├── test/                        plugin, port index and unit tests
└── docs/                        offline port page and AGENT_COMMENTS.md
```

## Build and test

Inside a MoveIt Pro 10.x image, from a workspace containing only this package or several packages:

```bash
source /opt/ros/jazzy/setup.bash
source /opt/overlay_ws/install/setup.bash
colcon build --packages-select control_interface_behaviors --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select control_interface_behaviors --executor sequential
colcon test-result --verbose
```

The plugin test loads this package's installed loader and creates every registered Behavior. The index
test checks that its documented IDs and ports match the code. These tests need no sibling packages.

When adding a Behavior, update this package's sources, dependencies and registration function, add tests,
then [regenerate the index](docs/behavior_index/README.md).

## Notes

- The class `AccessInterfaceValueFromGroup` registers as the ID `AccessInterfaceValue`. Objectives use the ID.
- The six Behaviors share one package because they only work together: one builds or receives the message that the next one reads.
