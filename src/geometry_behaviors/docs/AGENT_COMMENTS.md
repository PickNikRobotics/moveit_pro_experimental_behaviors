# Agent comments

Detailed comments by package-relative file and line. Keep entries short.

## include/geometry_behaviors/express_world_vector_in_link_frame.hpp

- line 15 (`ExpressWorldVectorInLinkFrame`): Why it exists: the velocity force controller reads every commanded
  twist in its own end-effector (grasp_link) frame and ignores `header.frame_id` (PublishVelocityForceCommand drops
  the header). To drive the wrist along a world-frame vector, such as straight down whatever the gripper tilt,
  convert the vector into the link frame first and pass that as the twist. `q_link` is the link orientation in
  world, for example from ExtractPoseOrientation.

## test/test_behavior_index.cpp

- One JSON field per line allows the comparison to ignore C++ type text. All IDs, metadata and
  other port fields must match. A `null` default means no default; `""` means an empty string.
