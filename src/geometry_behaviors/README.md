# Geometry Behaviors

**`geometry_behaviors` is a standalone ROS 2 package of MoveIt Pro Behaviors.**

Compute and test poses: align one pose axis or plane to a reference, make grids and rotated candidates of a pose, find the face pose of a box, and check whether a pose or a box is inside limits.


## Use this package

Requires MoveIt Pro 10.x on ROS 2 Jazzy. It uses the PickNik BehaviorTree.CPP fork.

1. Copy this directory to `<user_workspace>/src/geometry_behaviors/`. Keep its `LICENSE` and `NOTICE`.
2. Build with `moveit_pro build user_workspace`, then restart MoveIt Pro.
3. Use the Behavior IDs below in your Objectives.

`behavior_plugin.yaml` automatically registers `geometry_behaviors::GeometryBehaviorsLoader` in workspace configs.
No parent package, copied CMake helpers, manual registration call or sibling package is required.
MoveIt Pro, ROS 2 and system dependencies are declared in [package.xml](package.xml).

Remove the old `experimental_behaviors::ExperimentalBehaviorsLoader` entry from your config and remove
the old bulk package if it is present. Do not load another plugin with the same Behavior IDs; BT.CPP
rejects duplicate IDs. For a setup that does not read `behavior_plugin.yaml`, use this loader entry:

```yaml
objectives:
  behavior_loader_plugins:
    geometry_behaviors:
      - "geometry_behaviors::GeometryBehaviorsLoader"
```

## Behaviors

| Behavior ID | What it does |
| ----------- | ------------ |
| `CheckBoundingBoxInBounds` | Succeeds if a box (center pose and size) is inside world-frame minimum and maximum limits. |
| `ComputeCameraPoseFromTag` | Computes a camera pose in the world frame from one tag seen in the world frame and in the camera frame. |
| `ComputeRelativeAxisDot` | Outputs the absolute dot product of one named body axis of two poses: near 1 is parallel, near 0 is perpendicular. |
| `CreatePoseStampedGrid` | Makes a 3D grid of poses from one pose, with a count, a spacing and a rotation step per local axis. |
| `ExpressWorldVectorInLinkFrame` | Rotates a world-frame vector into a link frame, given the link orientation. |
| `GenerateAxisRotatedCandidates` | Makes copies of a pose, each rotated by one angle from a list about one body axis. |
| `GetBoundingBoxFacePose` | Computes the pose at the center of one face of an oriented box, with an optional outward offset. |
| `IsPoseInBox` | Succeeds if a pose is inside an oriented box. |
| `OrientPoseAxisToReferenceAxis` | Rotates a pose by the smallest rotation that aligns one of its axes with an axis of a reference pose. |
| `OrientPosePlaneToReferencePlane` | Rotates a pose by the smallest rotation that makes one of its planes parallel to a plane of a reference pose. |

Every port is documented in the [offline behavior index](docs/behavior_index/index.html).

## Package layout

```text
geometry_behaviors/
├── CMakeLists.txt, package.xml, README.md, LICENSE, NOTICE
├── behavior_plugin.yaml, geometry_behaviors_plugin_description.xml
├── include/geometry_behaviors/
├── src/                         Behaviors, helpers and this package's loader
├── test/                        plugin, port index and unit tests
└── docs/                        offline port page and AGENT_COMMENTS.md
```

## Build and test

Inside a MoveIt Pro 10.x image, from a workspace containing only this package or several packages:

```bash
source /opt/ros/jazzy/setup.bash
source /opt/overlay_ws/install/setup.bash
colcon build --packages-select geometry_behaviors --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select geometry_behaviors --executor sequential
colcon test-result --verbose
```

The plugin test loads this package's installed loader and creates every registered Behavior. The index
test checks that its documented IDs and ports match the code. These tests need no sibling packages.

When adding a Behavior, update this package's sources, dependencies and registration function, add tests,
then [regenerate the index](docs/behavior_index/README.md).
