# Agent comments

Detailed comments by package-relative file and line. Keep entries short.

## include/visualization_behaviors/clear_stale_contour_markers.hpp

- line 23 (`ClearStaleContourMarkers`): Marker layout of VisualizePath: one LINE_LIST (ns "<marker_name> - Line")
  plus one RGB triad per waypoint (ns "<marker_name> - Poses", three ids per waypoint). A later path with fewer
  waypoints overwrites the low ids only; an empty path publishes nothing, so the whole old path stays. The UI always
  honors a per-id DELETE. The DELETEs and the VisualizePath ADDs use disjoint ids, so their order does not matter as
  long as both run each pass.

## test/test_behavior_index.cpp

- One JSON field per line allows the comparison to ignore C++ type text. All IDs, metadata and
  other port fields must match. A `null` default means no default; `""` means an empty string.
