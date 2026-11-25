# Agent comments

Detailed comments by package-relative file and line. Keep entries short.

## include/mtc_behaviors/set_collisions_between_groups.hpp

- line 45 (`SetCollisionsBetweenGroups`): Why: the stock Behavior calls `acm.setEntry(list, list, allow)` with one
  list twice, which also sets the pairs the SRDF marks `<disable_collisions reason="Adjacent">`. When the tree later
  disallows, those pairs are enabled again and the next PlanToJointGoal fails on a self-collision between two
  adjacent wrist links.

## test/test_behavior_index.cpp

- One JSON field per line allows the comparison to ignore C++ type text. All IDs, metadata and
  other port fields must match. A `null` default means no default; `""` means an empty string.
