# Behavior index

**This offline page lists the IDs, metadata and ports registered by `visualization_behaviors`.**

Open [index.html](index.html) directly in a browser. No server or network is needed.
Search, filter, the package layout and the dark/light theme switch work offline.

## Update the data

[The index test](../../test/test_behavior_index.cpp) reads the real registration function and port
definitions. Build in a MoveIt Pro 10.x image, then run from the colcon workspace:

```bash
colcon build --packages-select visualization_behaviors --cmake-args -DBUILD_TESTING=ON
UPDATE_BEHAVIOR_INDEX=1 ./build/visualization_behaviors/test/test_behavior_index
```

Commit the changed `behavior_index_data.js`. Without the environment variable the test fails if IDs,
metadata, port names, directions, defaults or descriptions differ. C++ type text is not compared because
it can differ between builds. The data still records each type.

If working in the full repository, also run `python3 tools/update_behavior_index.py` from its root.
The package does not need that repository script to build, test or update its own page.
