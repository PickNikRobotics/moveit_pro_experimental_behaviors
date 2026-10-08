# Project agent memory

This file is the project's committed home for project-intrinsic agent knowledge: build, test, release, architecture, and sharp-edge notes that should travel with the code.

- Target MoveIt Pro version is the `image_tag` pin in `.github/workflows/CI.yaml` (CI builds Humble and Jazzy); the `package.xml` version is stale. Verify Behavior APIs against that Pro tag.
- Build and test inside the matching `picknikciuser/moveit-studio:<tag>-<distro>` image: `colcon build --packages-select experimental_behaviors`, then `colcon test --executor sequential` (as CI does).
- Format with clang-format 14.0.6 through `pre-commit run clang-format --files <files>` (`.pre-commit-config.yaml`); ament_clang_format and ament_clang_tidy were not installed in the 9.4.3-jazzy image, so those lint tests did not run there.
- Register every Behavior in `src/register_behaviors.cpp` and add it to `test/test_behavior_plugins.cpp`.
- Pro 9.x exposes the Behavior context as the protected `shared_resources_`, Pro 10.x only as `getBehaviorContext()`; most Behaviors here use the 9.x name, so the package does not build on 10.x. The localization Behaviors keep their own context pointer and build on both.
- Localization Behaviors live under `src/localization_behaviors/`, `include/experimental_behaviors/localization_behaviors/` and `test/localization_behaviors/`; user docs in `docs/localization_behaviors.md`. Pick Behavior IDs that do not clash with MoveIt Pro `main`: BT.CPP refuses a duplicate ID, which stops the whole plugin loading.

## Maintaining this file

Keep this file for knowledge useful to almost every future agent session in this project.
Do not repeat what the codebase already shows; point to the authoritative file or command instead.
Prefer rewriting or pruning existing entries over appending new ones.
When updating this file, preserve this bar for all agents and keep entries concise.
