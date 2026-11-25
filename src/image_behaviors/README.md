# Image Behaviors

**`image_behaviors` is a standalone ROS 2 package of MoveIt Pro Behaviors.**

Work with 2D segmentation masks (`moveit_studio_vision_msgs/Mask2D`) and `sensor_msgs/Image` images: find which mask holds a point, which points fall inside masks, pick the highest-score mask, draw points on an image, and save a depth image to a 16-bit PNG file.


## Use this package

Requires MoveIt Pro 10.x on ROS 2 Jazzy. It uses the PickNik BehaviorTree.CPP fork.

1. Copy this directory to `<user_workspace>/src/image_behaviors/`. Keep its `LICENSE` and `NOTICE`.
2. Build with `moveit_pro build user_workspace`, then restart MoveIt Pro.
3. Use the Behavior IDs below in your Objectives.

`behavior_plugin.yaml` automatically registers `image_behaviors::ImageBehaviorsLoader` in workspace configs.
No parent package, copied CMake helpers, manual registration call or sibling package is required.
MoveIt Pro, ROS 2 and system dependencies are declared in [package.xml](package.xml).

Remove the old `experimental_behaviors::ExperimentalBehaviorsLoader` entry from your config and remove
the old bulk package if it is present. Do not load another plugin with the same Behavior IDs; BT.CPP
rejects duplicate IDs. For a setup that does not read `behavior_plugin.yaml`, use this loader entry:

```yaml
objectives:
  behavior_loader_plugins:
    image_behaviors:
      - "image_behaviors::ImageBehaviorsLoader"
```

## Behaviors

| Behavior ID | What it does |
| ----------- | ------------ |
| `FindMaskIndexForPoint` | Outputs the index of the first `Mask2D` that contains a 2D point. |
| `FindPointsInMasks` | Outputs the indices of the 2D points that are inside any of a list of `Mask2D` masks. |
| `GetHighestConfidenceMask` | Picks the `Mask2D` with the highest confidence score from a list. |
| `MarkPointsOnImage` | Draws a list of points on an image and publishes the result, for a quick visual check. |
| `SaveDepthImageToFile` | Saves a depth image to a 16-bit PNG file and keeps the depth encoding, unlike `SaveImageToFile`. |

Every port is documented in the [offline behavior index](docs/behavior_index/index.html).

## Package layout

```text
image_behaviors/
├── CMakeLists.txt, package.xml, README.md, LICENSE, NOTICE
├── behavior_plugin.yaml, image_behaviors_plugin_description.xml
├── include/image_behaviors/
├── src/                         Behaviors, helpers and this package's loader
├── test/                        plugin, port index and unit tests
└── docs/                        offline port page and AGENT_COMMENTS.md
```

## Build and test

Inside a MoveIt Pro 10.x image, from a workspace containing only this package or several packages:

```bash
source /opt/ros/jazzy/setup.bash
source /opt/overlay_ws/install/setup.bash
colcon build --packages-select image_behaviors --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select image_behaviors --executor sequential
colcon test-result --verbose
```

The plugin test loads this package's installed loader and creates every registered Behavior. The index
test checks that its documented IDs and ports match the code. These tests need no sibling packages.

When adding a Behavior, update this package's sources, dependencies and registration function, add tests,
then [regenerate the index](docs/behavior_index/README.md).
