# AI Code Assistant Instructions for this repository

> `AGENTS.md` is the single source of instructions for every AI assistant. `CLAUDE.md` is a symlink to it.
> Edit `AGENTS.md` only.

## Package layout

- Each `src/*_behaviors/` directory is an independent ROS 2 package. Copy a package into a user workspace's
  `src/` directory without the rest of this repository.
- The repository root is a colcon workspace, not a ROS package. Keep every package under `src/`; the shared CI
  workflow runs `rosdep install --from-paths src`. Do not add a root `package.xml`, a shared loader or shared CMake helpers.
- Each package owns its manifest, library, C++ namespace, exported headers, loader, `behavior_plugin.yaml`, tests,
  license notices and offline port documentation. No package may include or depend on a sibling package.
- Detailed comments live in each package's `docs/AGENT_COMMENTS.md`, organized by package-relative file and line.
- `docs/behavior_index/` combines the packages' generated indexes. Run `python3 tools/update_behavior_index.py`
  after updating a package index. Root checks: `python3 test/test_package_isolation.py -v`.

## Target and validation

MoveIt Pro 10.x on ROS 2 Jazzy only. CI builds against `picknikciuser/moveit-pro:10.2.1-jazzy`.
Link `behaviortree_cpp_picknik`, never stock `behaviortree_cpp`.

Build in a MoveIt Pro image, never on the Mac host:

```bash
source /opt/ros/jazzy/setup.bash
source /opt/overlay_ws/install/setup.bash
colcon build --cmake-args -DBUILD_TESTING=ON
colcon test --executor sequential
colcon test-result --verbose
```

Also build each package alone in a clean workspace when changing its dependencies or exports.
Run `pre-commit run --all-files` before a commit. It uses the repository's clang-format rules.

## Behaviors, Objectives and comments

- Verify every Behavior ID and port against local C++ source before using it in XML. Never invent them.
- Parse every XML file you change. XML comment bodies must not contain a double hyphen.
- Inline comments: two lines is the target, four the hard limit. Put detail in the package's `docs/AGENT_COMMENTS.md`.
- In a doxygen block, each element has its own limit. `@details` may have three short paragraphs. Port tables are
  allowed and must match the code. Never repeat a tuned value in a comment.

## Licensing practice

- Every repo has a `LICENSE` with the BSD 3-Clause License from `moveit_pro_example_ws`, and a `NOTICE` that lists
  each explicitly included external dependency with its source URL and commit ID.
- Copy each explicitly included dependency's licence to `src/external_dependencies/licenses/LICENSE-<source>`.
- Accepted licences are Apache, BSD, MIT and Boost Software License 1.0. If a licence is unclear, do not block the
  include; tell the user. This does not approve other known licences.
- Do not add recursive licence inventories, duplicate licence copies or dedicated licensing tests solely for libraries
  encapsulated by an SDK. Use its supplied third-party notices when they cover the distributed parts; retain required
  licence and copyright notices in delivered source or images.
- Encapsulated licences must be at least as permissive as and compatible with the outer package's distribution. This
  is a selection requirement; the outer licence alone does not prove it or replace required third-party notices.
- Each copyable package carries its own `LICENSE` and `NOTICE`. Linked MoveIt Pro and ROS packages are not vendored.
