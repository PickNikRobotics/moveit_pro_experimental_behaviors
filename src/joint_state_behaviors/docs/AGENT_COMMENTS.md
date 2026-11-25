# Agent comments

Detailed comments by package-relative file and line. Keep entries short.

## include/joint_state_behaviors/filter_robot_state_to_joints.hpp

- line 33 (`FilterRobotStateToJoints`): Why it matters: the point-to-point planner without collision checking is the
  only way out once the start state is already in collision. A waypoint saved with every joint of the robot is not a
  valid target for it, so that escape has no reachable target unless the waypoint is filtered first.

## include/joint_state_behaviors/robot_state_from_pose_axis.hpp

- line 34 (`RobotStateFromPoseAxis`): Why RobotState: it is the type that `Move to Joint State` consumes and that
  CreateRobotState and RetrieveWaypoint produce. BT.CPP resolves a subtree port to one type across the whole tree,
  so mixing in a sensor_msgs::JointState fails at tree creation with a port-type conflict.

## test/test_behavior_index.cpp

- One JSON field per line allows the comparison to ignore C++ type text. All IDs, metadata and
  other port fields must match. A `null` default means no default; `""` means an empty string.
