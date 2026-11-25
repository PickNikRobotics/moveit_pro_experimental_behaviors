# Point cloud Behaviors

**`point_cloud_behaviors` is a standalone ROS 2 package of MoveIt Pro Behaviors.**

Work on `sensor_msgs/PointCloud2` clouds with PCL: remove outliers or the points inside a 3D mask, count points and voxels, mirror a cloud across a plane, rebuild a surface, find the closest point to a pose, clear the time stamp so TF uses the latest transforms, crop a cloud to a camera view, and load the newest saved cloud.


## Use this package

Requires MoveIt Pro 10.x on ROS 2 Jazzy. It uses the PickNik BehaviorTree.CPP fork.

1. Copy this directory to `<user_workspace>/src/point_cloud_behaviors/`. Keep its `LICENSE` and `NOTICE`.
2. Build with `moveit_pro build user_workspace`, then restart MoveIt Pro.
3. Use the Behavior IDs below in your Objectives.

`behavior_plugin.yaml` automatically registers `point_cloud_behaviors::PointCloudBehaviorsLoader` in workspace configs.
No parent package, copied CMake helpers, manual registration call or sibling package is required.
MoveIt Pro, ROS 2 and system dependencies are declared in [package.xml](package.xml).

Remove the old `experimental_behaviors::ExperimentalBehaviorsLoader` entry from your config and remove
the old bulk package if it is present. Do not load another plugin with the same Behavior IDs; BT.CPP
rejects duplicate IDs. For a setup that does not read `behavior_plugin.yaml`, use this loader entry:

```yaml
objectives:
  behavior_loader_plugins:
    point_cloud_behaviors:
      - "point_cloud_behaviors::PointCloudBehaviorsLoader"
```

## Behaviors

| Behavior ID | What it does |
| ----------- | ------------ |
| `ClearPointCloudTimestamp` | Sets the header stamp of a cloud to zero, so TF lookups use the latest transforms. Use it before an octomap update of a cloud that is not new. |
| `CountPointsInCloud` | Counts the finite points of a cloud and the voxels they fill. It only measures; it applies no threshold. |
| `CropPointCloudToCameraFrustum` | Keeps only the points inside the view volume of a camera, between a near and a far limit along the optical axis. |
| `ExcludeMask3DFromPointCloud` | Removes the points that are inside a `Mask3D` from a cloud. |
| `GetClosestPointPose` | Finds the cloud point closest to a pose and outputs a pose there, oriented by the local surface normal. |
| `LoadLatestPointCloudFromFile` | Loads the newest `.pcd` file in a directory whose name starts with a prefix, for example the last file of `SavePointCloudToFile`. |
| `MirrorPointCloud` | Reflects a cloud across a plane, for example to complete a scan of a symmetric object. |
| `PclNnFilter` | Removes the points that have fewer than N neighbors inside a radius (radius outlier filter). |
| `PoissonReconstruct` | Rebuilds a closed surface from a cloud with Poisson reconstruction, and colors measured and filled points differently. |
| `ReconstructSurface` | Smooths a cloud and fills small holes with moving least squares (MLS). |

Every port is documented in the [offline behavior index](docs/behavior_index/index.html).

## Package layout

```text
point_cloud_behaviors/
├── CMakeLists.txt, package.xml, README.md, LICENSE, NOTICE
├── behavior_plugin.yaml, point_cloud_behaviors_plugin_description.xml
├── include/point_cloud_behaviors/
├── src/                         Behaviors, helpers and this package's loader
├── test/                        plugin, port index and unit tests
└── docs/                        offline port page and AGENT_COMMENTS.md
```

## Build and test

Inside a MoveIt Pro 10.x image, from a workspace containing only this package or several packages:

```bash
source /opt/ros/jazzy/setup.bash
source /opt/overlay_ws/install/setup.bash
colcon build --packages-select point_cloud_behaviors --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select point_cloud_behaviors --executor sequential
colcon test-result --verbose
```

The plugin test loads this package's installed loader and creates every registered Behavior. The index
test checks that its documented IDs and ports match the code. These tests need no sibling packages.

When adding a Behavior, update this package's sources, dependencies and registration function, add tests,
then [regenerate the index](docs/behavior_index/README.md).

## Notes

- `PclNnFilter` is the Behavior ID; its C++ class is `ExampleRadiusNeighborsFilter`.
- `CountPointsInCloud` and `PclNnFilter` use point helpers from the MoveIt Pro `moveit_studio_vision` library.
- `CMakeLists.txt` enables the C language and finds MPI. PCL links the VTK of Ubuntu, and VTK needs the `MPI::MPI_C` target. See this package's `docs/AGENT_COMMENTS.md`.
- `CropPointCloudToCameraFrustum` reads the x, y and z fields straight from the `PointCloud2` message (private helper `src/point_cloud_xyz.hpp`), so every other field is kept.
- `test/` tests `CropPointCloudToCameraFrustum` on a small cloud and `LoadLatestPointCloudFromFile` on files in a temporary directory.
