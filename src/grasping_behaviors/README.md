# Grasping Behaviors

**`grasping_behaviors` is a standalone ROS 2 package of MoveIt Pro Behaviors.**

Work with `moveit_studio_vision_msgs/GraspableObject` messages: filter them by size, read their fitted shape primitive, label a grasp as vertical or horizontal, and make grasp pose candidates for cylinders and spheres.


## Use this package

Requires MoveIt Pro 10.x on ROS 2 Jazzy. It uses the PickNik BehaviorTree.CPP fork.

1. Copy this directory to `<user_workspace>/src/grasping_behaviors/`. Keep its `LICENSE` and `NOTICE`.
2. Build with `moveit_pro build user_workspace`, then restart MoveIt Pro.
3. Use the Behavior IDs below in your Objectives.

`behavior_plugin.yaml` automatically registers `grasping_behaviors::GraspingBehaviorsLoader` in workspace configs.
No parent package, copied CMake helpers, manual registration call or sibling package is required.
MoveIt Pro, ROS 2 and system dependencies are declared in [package.xml](package.xml).

Remove the old `experimental_behaviors::ExperimentalBehaviorsLoader` entry from your config and remove
the old bulk package if it is present. Do not load another plugin with the same Behavior IDs; BT.CPP
rejects duplicate IDs. For a setup that does not read `behavior_plugin.yaml`, use this loader entry:

```yaml
objectives:
  behavior_loader_plugins:
    grasping_behaviors:
      - "grasping_behaviors::GraspingBehaviorsLoader"
```

## Behaviors

| Behavior ID | What it does |
| ----------- | ------------ |
| `DeterminePickAlignment` | Labels a grasp as vertical or horizontal relative to the shape primitive fitted to the object. |
| `ExtractGraspableObjectPrimitive` | Outputs the `SolidPrimitive` of a `GraspableObject`. |
| `FilterGraspableObjects` | Keeps the `GraspableObject` messages under a maximum size and volume, sorts them highest first, and shows markers. |
| `GenerateGraspCandidates` | Makes grasp pose candidates for a `GraspableObject` that is a cylinder or a sphere. |

Every port is documented in the [offline behavior index](docs/behavior_index/index.html).

## Package layout

```text
grasping_behaviors/
├── CMakeLists.txt, package.xml, README.md, LICENSE, NOTICE
├── behavior_plugin.yaml, grasping_behaviors_plugin_description.xml
├── include/grasping_behaviors/
├── src/                         Behaviors, helpers and this package's loader
├── test/                        plugin, port index and unit tests
└── docs/                        offline port page and AGENT_COMMENTS.md
```

## Build and test

Inside a MoveIt Pro 10.x image, from a workspace containing only this package or several packages:

```bash
source /opt/ros/jazzy/setup.bash
source /opt/overlay_ws/install/setup.bash
colcon build --packages-select grasping_behaviors --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select grasping_behaviors --executor sequential
colcon test-result --verbose
```

The plugin test loads this package's installed loader and creates every registered Behavior. The index
test checks that its documented IDs and ports match the code. These tests need no sibling packages.

When adding a Behavior, update this package's sources, dependencies and registration function, add tests,
then [regenerate the index](docs/behavior_index/README.md).
