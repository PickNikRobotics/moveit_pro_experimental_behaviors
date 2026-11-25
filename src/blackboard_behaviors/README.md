# Blackboard Behaviors

**`blackboard_behaviors` is a standalone ROS 2 package of MoveIt Pro Behaviors.**

Read and write a blackboard entry whose key the tree computes at run time, for example with a Script node.


## Use this package

Requires MoveIt Pro 10.x on ROS 2 Jazzy. It uses the PickNik BehaviorTree.CPP fork.

1. Copy this directory to `<user_workspace>/src/blackboard_behaviors/`. Keep its `LICENSE` and `NOTICE`.
2. Build with `moveit_pro build user_workspace`, then restart MoveIt Pro.
3. Use the Behavior IDs below in your Objectives.

`behavior_plugin.yaml` automatically registers `blackboard_behaviors::BlackboardBehaviorsLoader` in workspace configs.
No parent package, copied CMake helpers, manual registration call or sibling package is required.
MoveIt Pro, ROS 2 and system dependencies are declared in [package.xml](package.xml).

Remove the old `experimental_behaviors::ExperimentalBehaviorsLoader` entry from your config and remove
the old bulk package if it is present. Do not load another plugin with the same Behavior IDs; BT.CPP
rejects duplicate IDs. For a setup that does not read `behavior_plugin.yaml`, use this loader entry:

```yaml
objectives:
  behavior_loader_plugins:
    blackboard_behaviors:
      - "blackboard_behaviors::BlackboardBehaviorsLoader"
```

## Behaviors

| Behavior ID | What it does |
| ----------- | ------------ |
| `GetBlackboardByKey` | Copies the value of the blackboard entry named by `key` to the `value` output port. |
| `SetBlackboardByKey` | Copies the `value` input port to the blackboard entry named by `key`, and keeps the type of the value. |

Every port is documented in the [offline behavior index](docs/behavior_index/index.html).

## Package layout

```text
blackboard_behaviors/
├── CMakeLists.txt, package.xml, README.md, LICENSE, NOTICE
├── behavior_plugin.yaml, blackboard_behaviors_plugin_description.xml
├── include/blackboard_behaviors/
├── src/                         Behaviors, helpers and this package's loader
├── test/                        plugin, port index and unit tests
└── docs/                        offline port page and AGENT_COMMENTS.md
```

## Build and test

Inside a MoveIt Pro 10.x image, from a workspace containing only this package or several packages:

```bash
source /opt/ros/jazzy/setup.bash
source /opt/overlay_ws/install/setup.bash
colcon build --packages-select blackboard_behaviors --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select blackboard_behaviors --executor sequential
colcon test-result --verbose
```

The plugin test loads this package's installed loader and creates every registered Behavior. The index
test checks that its documented IDs and ports match the code. These tests need no sibling packages.

When adding a Behavior, update this package's sources, dependencies and registration function, add tests,
then [regenerate the index](docs/behavior_index/README.md).

## Notes

- `SetBlackboardByKey` exists because BT.CPP's own `SetBlackboard` can empty a value of type `AnyTypeAllowed`. The header `set_blackboard_by_key.hpp` gives the reason.
