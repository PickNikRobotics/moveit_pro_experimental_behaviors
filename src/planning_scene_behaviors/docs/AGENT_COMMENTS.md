# Agent comments

Detailed comments by package-relative file and line. Keep entries short.

## include/planning_scene_behaviors/add_colored_virtual_object_to_planning_scene.hpp

- line 43 (`AddColoredVirtualObjectToPlanningScene`): The chain it replaces: Convert CollisionObject To
  GraspableObject, UnpackGraspableObjectMessage, CreateColorRGBAMessage, CreateObjectColorMessage,
  CreateGraspableObjectMessage, AddVirtualObjectToPlanningScene. Allowed collision matrix rule, as in the product
  Behavior: forbid collisions with non-virtual objects, allow collisions with other virtual objects.

## test/test_add_colored_virtual_object_to_planning_scene.cpp

- line 7: Cases covered: RGBA out of range (error); alpha at both ends of the unit interval (accepted); an id with
  "virtual_" in the middle, for the new object and for existing ACM entries (regression of a substring check); a
  clean snapshot gives one self-allowed, default-denied ACM row; a virtual obstacle in the snapshot is allowed and a
  non-virtual one forbidden; a snapshot that already has the prefixed id still returns SUCCESS (the helper warns, it
  does not skip).

## test/test_behavior_index.cpp

- One JSON field per line allows the comparison to ignore C++ type text. All IDs, metadata and
  other port fields must match. A `null` default means no default; `""` means an empty string.
