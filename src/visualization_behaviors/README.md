# Visualization Behaviors

**`visualization_behaviors` is a standalone ROS 2 package of MoveIt Pro Behaviors.**

Publish `visualization_msgs/MarkerArray` messages from a tree: a prebuilt array, markers made from poses with a color and scale, and deletes for markers that a longer earlier path left behind.


## Use this package

Requires MoveIt Pro 10.x on ROS 2 Jazzy. It uses the PickNik BehaviorTree.CPP fork.

1. Copy this directory to `<user_workspace>/src/visualization_behaviors/`. Keep its `LICENSE` and `NOTICE`.
2. Build with `moveit_pro build user_workspace`, then restart MoveIt Pro.
3. Use the Behavior IDs below in your Objectives.

`behavior_plugin.yaml` automatically registers `visualization_behaviors::VisualizationBehaviorsLoader` in workspace configs.
No parent package, copied CMake helpers, manual registration call or sibling package is required.
MoveIt Pro, ROS 2 and system dependencies are declared in [package.xml](package.xml).

Remove the old `experimental_behaviors::ExperimentalBehaviorsLoader` entry from your config and remove
the old bulk package if it is present. Do not load another plugin with the same Behavior IDs; BT.CPP
rejects duplicate IDs. For a setup that does not read `behavior_plugin.yaml`, use this loader entry:

```yaml
objectives:
  behavior_loader_plugins:
    visualization_behaviors:
      - "visualization_behaviors::VisualizationBehaviorsLoader"
```

## Behaviors

| Behavior ID | What it does |
| ----------- | ------------ |
| `ClearStaleContourMarkers` | Deletes, by marker ID, the markers that an earlier and longer `VisualizePath` call left in a namespace. |
| `PublishMarkerArray` | Publishes a prebuilt `MarkerArray` on a topic. |
| `PublishModifyMarkers` | Publishes, modifies or deletes markers at a list of poses, with a type, color and scale. |

Every port is documented in the [offline behavior index](docs/behavior_index/index.html).

## Package layout

```text
visualization_behaviors/
├── CMakeLists.txt, package.xml, README.md, LICENSE, NOTICE
├── behavior_plugin.yaml, visualization_behaviors_plugin_description.xml
├── include/visualization_behaviors/
├── src/                         Behaviors, helpers and this package's loader
├── test/                        plugin, port index and unit tests
└── docs/                        offline port page and AGENT_COMMENTS.md
```

## Build and test

Inside a MoveIt Pro 10.x image, from a workspace containing only this package or several packages:

```bash
source /opt/ros/jazzy/setup.bash
source /opt/overlay_ws/install/setup.bash
colcon build --packages-select visualization_behaviors --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select visualization_behaviors --executor sequential
colcon test-result --verbose
```

The plugin test loads this package's installed loader and creates every registered Behavior. The index
test checks that its documented IDs and ports match the code. These tests need no sibling packages.

When adding a Behavior, update this package's sources, dependencies and registration function, add tests,
then [regenerate the index](docs/behavior_index/README.md).
