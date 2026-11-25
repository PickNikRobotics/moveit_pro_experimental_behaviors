# Joint state Behaviors

**`joint_state_behaviors` is a standalone ROS 2 package of MoveIt Pro Behaviors.**

Read, set and filter joint values in `sensor_msgs/JointState`, `moveit_studio_agent_msgs/RobotState` and `moveit_msgs/RobotState` messages, check that one joint is inside a range, and measure the distance between two joint states.


## Use this package

Requires MoveIt Pro 10.x on ROS 2 Jazzy. It uses the PickNik BehaviorTree.CPP fork.

1. Copy this directory to `<user_workspace>/src/joint_state_behaviors/`. Keep its `LICENSE` and `NOTICE`.
2. Build with `moveit_pro build user_workspace`, then restart MoveIt Pro.
3. Use the Behavior IDs below in your Objectives.

`behavior_plugin.yaml` automatically registers `joint_state_behaviors::JointStateBehaviorsLoader` in workspace configs.
No parent package, copied CMake helpers, manual registration call or sibling package is required.
MoveIt Pro, ROS 2 and system dependencies are declared in [package.xml](package.xml).

Remove the old `experimental_behaviors::ExperimentalBehaviorsLoader` entry from your config and remove
the old bulk package if it is present. Do not load another plugin with the same Behavior IDs; BT.CPP
rejects duplicate IDs. For a setup that does not read `behavior_plugin.yaml`, use this loader entry:

```yaml
objectives:
  behavior_loader_plugins:
    joint_state_behaviors:
      - "joint_state_behaviors::JointStateBehaviorsLoader"
```

## Behaviors

| Behavior ID | What it does |
| ----------- | ------------ |
| `ComputeJointStateDistance` | Computes the joint-space distance between two joint states over the joints you name: Euclidean norm and largest single-joint difference. |
| `CreateEmptyMultiDOFJointState` | Outputs an empty `MultiDOFJointState` message, for ports that need one. |
| `FilterRobotJointState` | Keeps only the joints of one planning group in a `RobotJointState` message. |
| `FilterRobotStateToJoints` | Keeps only the named joints in a `RobotState` message. |
| `GetJointValueFromRobotJointState` | Reads the value of one joint from a `RobotJointState` message. |
| `IsJointPositionInRange` | Succeeds if one joint position in a planning scene is inside a minimum and maximum. Useful as a grasp check. |
| `RobotStateFromPoseAxis` | Makes a one-joint `RobotState` whose value is one translation component of a pose, for example to line up a prismatic axis. |
| `SetJointStateValues` | Overwrites the values of named joints in a `JointState` message. |
| `SetJointValueInRobotJointState` | Sets the value of one joint in a `RobotJointState` message. |

Every port is documented in the [offline behavior index](docs/behavior_index/index.html).

## Package layout

```text
joint_state_behaviors/
├── CMakeLists.txt, package.xml, README.md, LICENSE, NOTICE
├── behavior_plugin.yaml, joint_state_behaviors_plugin_description.xml
├── include/joint_state_behaviors/
├── src/                         Behaviors, helpers and this package's loader
├── test/                        plugin, port index and unit tests
└── docs/                        offline port page and AGENT_COMMENTS.md
```

## Build and test

Inside a MoveIt Pro 10.x image, from a workspace containing only this package or several packages:

```bash
source /opt/ros/jazzy/setup.bash
source /opt/overlay_ws/install/setup.bash
colcon build --packages-select joint_state_behaviors --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select joint_state_behaviors --executor sequential
colcon test-result --verbose
```

The plugin test loads this package's installed loader and creates every registered Behavior. The index
test checks that its documented IDs and ports match the code. These tests need no sibling packages.

When adding a Behavior, update this package's sources, dependencies and registration function, add tests,
then [regenerate the index](docs/behavior_index/README.md).

## Notes

- `test/test_is_joint_position_in_range.cpp` tests the joint lookup that `IsJointPositionInRange` uses.
- `test/test_compute_joint_state_distance.cpp` tests the distance math of `ComputeJointStateDistance`.
