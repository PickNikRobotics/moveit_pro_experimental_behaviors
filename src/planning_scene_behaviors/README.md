# Planning scene Behaviors

**`planning_scene_behaviors` is a standalone ROS 2 package of MoveIt Pro Behaviors.**

Work with the MoveIt planning scene: add a colored virtual object, apply a scene diff, copy a robot joint state into a scene message, read the allowed collision matrix or an attached object pose, grow the octomap, make a URDF for a primitive shape, and check one joint state for collision.


## Use this package

Requires MoveIt Pro 10.x on ROS 2 Jazzy. It uses the PickNik BehaviorTree.CPP fork.

1. Copy this directory to `<user_workspace>/src/planning_scene_behaviors/`. Keep its `LICENSE` and `NOTICE`.
2. Build with `moveit_pro build user_workspace`, then restart MoveIt Pro.
3. Use the Behavior IDs below in your Objectives.

`behavior_plugin.yaml` automatically registers `planning_scene_behaviors::PlanningSceneBehaviorsLoader` in workspace configs.
No parent package, copied CMake helpers, manual registration call or sibling package is required.
MoveIt Pro, ROS 2 and system dependencies are declared in [package.xml](package.xml).

Remove the old `experimental_behaviors::ExperimentalBehaviorsLoader` entry from your config and remove
the old bulk package if it is present. Do not load another plugin with the same Behavior IDs; BT.CPP
rejects duplicate IDs. For a setup that does not read `behavior_plugin.yaml`, use this loader entry:

```yaml
objectives:
  behavior_loader_plugins:
    planning_scene_behaviors:
      - "planning_scene_behaviors::PlanningSceneBehaviorsLoader"
```

## Behaviors

| Behavior ID | What it does |
| ----------- | ------------ |
| `AddColoredVirtualObjectToPlanningScene` | Adds a `CollisionObject` as a virtual object with an RGBA color, in one `/apply_planning_scene` call. |
| `AddRobotJointStateToPlanningSceneMsg` | Copies the joint values of a `RobotJointState` into the robot state of a `PlanningScene` message. |
| `ApplyPlanningSceneDiff` | Sends a `PlanningScene` diff to the `/apply_planning_scene` service. |
| `GenerateShapeUrdf` | Makes a URDF string for one box, sphere or cylinder. |
| `GetAttachedCollisionObjectPoseStamped` | Outputs the pose of an attached collision object from a `PlanningScene` message. |
| `GetPlanningSceneACMOnly` | Gets only the allowed collision matrix of the current planning scene, which is faster than the full scene. |
| `InflateOctomap` | Grows the occupied voxels of the octomap in a `PlanningScene` message by a radius, as a safety margin. |
| `IsJointStateInCollision` | Fails if an arm joint state, with extra end-of-arm tool (EOAT) joint values, is in collision in a planning scene. |

Every port is documented in the [offline behavior index](docs/behavior_index/index.html).

## Package layout

```text
planning_scene_behaviors/
├── CMakeLists.txt, package.xml, README.md, LICENSE, NOTICE
├── behavior_plugin.yaml, planning_scene_behaviors_plugin_description.xml
├── include/planning_scene_behaviors/
├── src/                         Behaviors, helpers and this package's loader
├── test/                        plugin, port index and unit tests
└── docs/                        offline port page and AGENT_COMMENTS.md
```

## Build and test

Inside a MoveIt Pro 10.x image, from a workspace containing only this package or several packages:

```bash
source /opt/ros/jazzy/setup.bash
source /opt/overlay_ws/install/setup.bash
colcon build --packages-select planning_scene_behaviors --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select planning_scene_behaviors --executor sequential
colcon test-result --verbose
```

The plugin test loads this package's installed loader and creates every registered Behavior. The index
test checks that its documented IDs and ports match the code. These tests need no sibling packages.

When adding a Behavior, update this package's sources, dependencies and registration function, add tests,
then [regenerate the index](docs/behavior_index/README.md).

## Notes

- Use `ApplyPlanningSceneDiff` after `InflateOctomap` to send the grown octomap to the planning scene.
- `test/` tests the request that `AddColoredVirtualObjectToPlanningScene` builds, and the URDF output of `GenerateShapeUrdf`.
