# Agent comments

Detailed comments by package-relative file and line. Keep entries short.

## src/grasp_candidate_generation.cpp

- line 42: End-face candidates: outward has Z along the outward end normal (gripper above the end face). Inward has
  Z along the inward end normal (gripper below the end face, pointing at the cylinder center); it is the outward
  pose flipped by a half turn about its local X axis, which keeps the frame right-handed.

## test/test_behavior_index.cpp

- One JSON field per line allows the comparison to ignore C++ type text. All IDs, metadata and
  other port fields must match. A `null` default means no default; `""` means an empty string.
