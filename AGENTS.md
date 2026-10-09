# Project agent memory

This file is the project's committed home for project-intrinsic agent knowledge: build, test, release, architecture, and sharp-edge notes that should travel with the code.

- Target MoveIt Pro version is the `image_tag` pin in `.github/workflows/CI.yaml` (CI builds Humble and Jazzy); the `package.xml` version is stale. Verify Behavior APIs against that Pro tag.
- Build and test inside the matching `picknikciuser/moveit-studio:<tag>-<distro>` image: `colcon build --packages-select experimental_behaviors`, then `colcon test --executor sequential` (as CI does).
- Format with clang-format 14.0.6 through `pre-commit run clang-format --files <files>` (`.pre-commit-config.yaml`); ament_clang_format and ament_clang_tidy were not installed in the 9.4.3-jazzy image, so those lint tests did not run there.
- Register every Behavior in `src/register_behaviors.cpp` and add it to `test/test_behavior_plugins.cpp`.
- Behavior groups are folders, not packages: `json_behaviors/` and `restful_behaviors/` under `src/`, `include/experimental_behaviors/` and `test/`, with user docs in `docs/<group>.md`. Keep `restful_behaviors/` free of JSON-group includes: `docs/restful_behaviors_copy_howto.html` tells users to copy its two files alone (into Pro 9.4 or 10.x).
- BT.CPP reads any port text wrapped in braces as a blackboard key, so a JSON object literal typed into a port is lost unless read raw (see `json_utils::getJsonText`).
- BT.CPP refuses to create a tree where a `std::string` port and an `int` port share a blackboard key; convert in a Script first (`text := '' .. code`).

## Maintaining this file

Keep this file for knowledge useful to almost every future agent session in this project.
Do not repeat what the codebase already shows; point to the authoritative file or command instead.
Prefer rewriting or pruning existing entries over appending new ones.
When updating this file, preserve this bar for all agents and keep entries concise.
