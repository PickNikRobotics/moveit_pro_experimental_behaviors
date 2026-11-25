# Agent comments

Detailed comments by package-relative file and line. Keep entries short.

## include/grasping_behaviors/generate_place_candidates.hpp

- line 71 (`GeneratePlaceCandidates`): The math, all in one frame. `T_grasp_object = T_grasp_at_pick^-1 * T_object_at_pick`
  stays fixed while the object is held. The desired object pose is `target_pose * translation(target_offset_xyz)`;
  the upside-down variant adds a half turn about its own X axis. Each candidate is
  `T_grasp = T_object_desired * T_grasp_object^-1`. Yaw turns are applied on the left, about the Z axis of the
  target pose through the place point, so the upside-down variant turns about the same axis as the upright one.
  object_pose and pick_pose may be in any frame, but in the same one; the outputs carry the frame of target_pose.

## src/grasp_candidate_generation.cpp

- line 42: End-face candidates: outward has Z along the outward end normal (gripper above the end face). Inward has
  Z along the inward end normal (gripper below the end face, pointing at the cylinder center); it is the outward
  pose flipped by a half turn about its local X axis, which keeps the frame right-handed.

## test/test_behavior_index.cpp

- One JSON field per line allows the comparison to ignore C++ type text. All IDs, metadata and
  other port fields must match. A `null` default means no default; `""` means an empty string.
