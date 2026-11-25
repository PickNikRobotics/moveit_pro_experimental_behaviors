# Hardware Behaviors

**`hardware_behaviors` is a standalone ROS 2 package of MoveIt Pro Behaviors.**

Change the lifecycle state of a ros2_control hardware component through the controller manager, the Behavior form of `ros2 control set_hardware_component_state`. Use it to bring hardware back after a driver error.


## Use this package

Requires MoveIt Pro 10.x on ROS 2 Jazzy. It uses the PickNik BehaviorTree.CPP fork.

1. Copy this directory to `<user_workspace>/src/hardware_behaviors/`. Keep its `LICENSE` and `NOTICE`.
2. Build with `moveit_pro build user_workspace`, then restart MoveIt Pro.
3. Use the Behavior IDs below in your Objectives.

`behavior_plugin.yaml` automatically registers `hardware_behaviors::HardwareBehaviorsLoader` in workspace configs.
No parent package, copied CMake helpers, manual registration call or sibling package is required.
MoveIt Pro, ROS 2 and system dependencies are declared in [package.xml](package.xml).

Remove the old `experimental_behaviors::ExperimentalBehaviorsLoader` entry from your config and remove
the old bulk package if it is present. Do not load another plugin with the same Behavior IDs; BT.CPP
rejects duplicate IDs. For a setup that does not read `behavior_plugin.yaml`, use this loader entry:

```yaml
objectives:
  behavior_loader_plugins:
    hardware_behaviors:
      - "hardware_behaviors::HardwareBehaviorsLoader"
```

## Behaviors

| Behavior ID | What it does |
| ----------- | ------------ |
| `SetHardwareComponentState` | Calls the controller manager `set_hardware_component_state` service to move one hardware component to active, inactive, unconfigured or finalized. |

Every port is documented in the [offline behavior index](docs/behavior_index/index.html).

## Package layout

```text
hardware_behaviors/
├── CMakeLists.txt, package.xml, README.md, LICENSE, NOTICE
├── behavior_plugin.yaml, hardware_behaviors_plugin_description.xml
├── include/hardware_behaviors/
├── src/                         Behaviors, helpers and this package's loader
├── test/                        plugin, port index and unit tests
└── docs/                        offline port page and AGENT_COMMENTS.md
```

## Build and test

Inside a MoveIt Pro 10.x image, from a workspace containing only this package or several packages:

```bash
source /opt/ros/jazzy/setup.bash
source /opt/overlay_ws/install/setup.bash
colcon build --packages-select hardware_behaviors --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select hardware_behaviors --executor sequential
colcon test-result --verbose
```

The plugin test loads this package's installed loader and creates every registered Behavior. The index
test checks that its documented IDs and ports match the code. These tests need no sibling packages.

When adding a Behavior, update this package's sources, dependencies and registration function, add tests,
then [regenerate the index](docs/behavior_index/README.md).

## Notes

- Asking for the state that a component is already in fails. Wrap a defensive `inactive` step in `ForceSuccess`.
