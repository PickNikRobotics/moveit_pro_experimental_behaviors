# YAML Behaviors

**`yaml_behaviors` is a standalone ROS 2 package of MoveIt Pro Behaviors.**

Change a value or append to a list in a YAML file on disk. Each write is atomic and holds a file lock, so a reader never sees a half-written file and two writers do not lose each other's change.


## Use this package

Requires MoveIt Pro 10.x on ROS 2 Jazzy. It uses the PickNik BehaviorTree.CPP fork.

1. Copy this directory to `<user_workspace>/src/yaml_behaviors/`. Keep its `LICENSE` and `NOTICE`.
2. Build with `moveit_pro build user_workspace`, then restart MoveIt Pro.
3. Use the Behavior IDs below in your Objectives.

`behavior_plugin.yaml` automatically registers `yaml_behaviors::YamlBehaviorsLoader` in workspace configs.
No parent package, copied CMake helpers, manual registration call or sibling package is required.
MoveIt Pro, ROS 2 and system dependencies are declared in [package.xml](package.xml).

Remove the old `experimental_behaviors::ExperimentalBehaviorsLoader` entry from your config and remove
the old bulk package if it is present. Do not load another plugin with the same Behavior IDs; BT.CPP
rejects duplicate IDs. For a setup that does not read `behavior_plugin.yaml`, use this loader entry:

```yaml
objectives:
  behavior_loader_plugins:
    yaml_behaviors:
      - "yaml_behaviors::YamlBehaviorsLoader"
```

## Behaviors

| Behavior ID | What it does |
| ----------- | ------------ |
| `WriteYamlValue` | Sets the value at a key path (up to five keys) in a YAML file. |
| `AppendYamlListItem` | Appends a value to the list at a key path (up to five keys) in a YAML file. |

Every port is documented in the [offline behavior index](docs/behavior_index/index.html).

## Package layout

```text
yaml_behaviors/
├── CMakeLists.txt, package.xml, README.md, LICENSE, NOTICE
├── behavior_plugin.yaml, yaml_behaviors_plugin_description.xml
├── include/yaml_behaviors/
├── src/                         Behaviors, helpers and this package's loader
├── test/                        plugin, port index and unit tests
└── docs/                        offline port page and AGENT_COMMENTS.md
```

## Build and test

Inside a MoveIt Pro 10.x image, from a workspace containing only this package or several packages:

```bash
source /opt/ros/jazzy/setup.bash
source /opt/overlay_ws/install/setup.bash
colcon build --packages-select yaml_behaviors --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select yaml_behaviors --executor sequential
colcon test-result --verbose
```

The plugin test loads this package's installed loader and creates every registered Behavior. The index
test checks that its documented IDs and ports match the code. These tests need no sibling packages.

When adding a Behavior, update this package's sources, dependencies and registration function, add tests,
then [regenerate the index](docs/behavior_index/README.md).

## Notes

- `file_path` accepts an absolute path, `${VAR}` and a leading `~`, or a `package://<package>/<path>` URL.
- The shared helpers `atomic_file_write.hpp` and `path_expansion.hpp` are in `include/`; `src/yaml_key_path.hpp` is private to this package.
- `test/test_yaml_file_helpers.cpp` tests the helpers. The package URL case resolves the installed `yaml_behaviors` package.
