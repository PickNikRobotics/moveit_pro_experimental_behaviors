# Agent comments

Detailed comments by package-relative file and line. Keep entries short.

## include/yaml_behaviors/read_yaml_value.hpp

- line 36: `optional`: only an absent key is tolerated. A missing package or file, unparsable YAML, a parent that
  is not a map, an ambiguous case-insensitive match and a non-scalar value all still fail. A Fallback at the call
  site would hide those too, so a malformed config would look the same as an unconfigured one.

## test/test_behavior_index.cpp

- One JSON field per line allows the comparison to ignore C++ type text. All IDs, metadata and
  other port fields must match. A `null` default means no default; `""` means an empty string.

## CMakeLists.txt

- The yaml-cpp target is linked privately because public headers do not use it. Tests of the
  private key lookup helper link it explicitly. No parent CMake variables are needed.
