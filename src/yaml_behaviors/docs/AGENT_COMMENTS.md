# Agent comments

Detailed comments by package-relative file and line. Keep entries short.



## test/test_behavior_index.cpp

- One JSON field per line allows the comparison to ignore C++ type text. All IDs, metadata and
  other port fields must match. A `null` default means no default; `""` means an empty string.

## CMakeLists.txt

- The yaml-cpp target is linked privately because public headers do not use it. Tests of the
  private key lookup helper link it explicitly. No parent CMake variables are needed.
