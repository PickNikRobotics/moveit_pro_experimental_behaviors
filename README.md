# Experimental Behaviors

`experimental_behaviors::ExperimentalBehaviorsLoader` supplies 13 Behavior IDs. The default configuration registers all 13, including on MoveIt Pro 9.4. No Behavior is removed by installing a newer Runtime alone.

## Runtime compatibility

Rebuild this package against the matching MoveIt Pro Runtime and its BehaviorTree.CPP dependency. On Humble the package uses `behaviortree_cpp`; on Jazzy it uses the `behaviortree_cpp_picknik` fork. A binary built for one Runtime is not a compatibility layer for another Runtime.

The package-local context adapter retains the constructor's `std::shared_ptr<BehaviorContext>`. It does not depend on the protected `shared_resources_` member in 9.4 or the context accessor in newer releases. The trajectory adapter selects the available RobotState position setter: a `std::vector<double>` on the legacy API, or an Eigen view of the same positions on the current API. The joint-bounds adapter normalizes the legacy and current member spellings for the same three position bounds. These adapters do not change Behavior ports or algorithms. The topic subscribers retain their leaf-owned futures required by `AsyncBehaviorBase`.

CI retains its MoveIt Pro 9.4.3 image default. The workflow's `image_tag` input selects a different Runtime for a separate compatibility run; the default lane alone does not establish compatibility with that Runtime.

## Staged Behavior IDs

Only these existing IDs are eligible for exclusion when their replacements are deployed:

- `CreateInterfaceValue`
- `CreateDynamicInterfaceGroupValues`
- `GetInterfaceValueFromGroup`
- `GetDynamicInterfaceGroupValues`
- `PublishDynamicInterfaceGroupValues`

The staged list does not establish that every Runtime release provides these core Behaviors. Exclude an ID only after verifying that the deployed core loader supplies that exact ID and that the replacement's port names, types, and behavior satisfy the Objectives that use it. A renamed replacement does not supply an unchanged ID.

## Startup configuration

`experimental_behaviors.excluded_behaviors` is a read-only string-array parameter on the node supplied to the loader through `BehaviorContext`. Its default is an empty array. Leave it unset or empty on MoveIt Pro 9.4 and on deployments without the matching core replacements.

The parameter belongs in the ROS parameter file for that node, not in Objective XML. A ROS parameter override for `CreateInterfaceValue`, after its core replacement is deployed, is:

```yaml
/**:
  ros__parameters:
    experimental_behaviors.excluded_behaviors:
      - CreateInterfaceValue
```

Keep existing robot parameters and the experimental loader enabled. Add only the deployed IDs to the array as each migration stage lands. Restart the Runtime after changing the array; it cannot be changed or undeclared on the running node. An already-declared parameter must also be a read-only string array; writable automatic declarations are rejected.

Registration runs sequentially on the Runtime's startup thread. Concurrent calls to the same node and factory are not supported.

The loader validates the entire array before registering any Behaviors. Unknown IDs, IDs outside the staged list, empty strings, duplicates, and non-string-array values throw `BT::RuntimeError` without partially registering this package. IDs are case-sensitive and are not trimmed.

For a valid array, the loader skips only the named experimental registrations. It neither inspects the factory to infer exclusions nor unregisters or replaces existing builders. The exclusion works whether the replacement loader runs before or after the experimental loader. An unconfigured ID collision remains a registration error.

This loader cannot detect a missing replacement after all loaders have finished. A valid exclusion succeeds even if no replacement has been registered yet. Deployment acceptance therefore requires checking the final Behavior inventory and validating the affected Objectives against the deployed Runtime. Successful experimental-loader registration alone does not establish replacement availability or Objective compatibility.

## Remaining Behaviors

These eight IDs remain available even when all five staged IDs are excluded:

- `GetPoseStampedFromTopic`
- `AccessInterfaceValue`
- `GetBlackboardByKey`
- `GetJointLimits`
- `SetBlackboardByKey`
- `TrajectoryToPath`
- `WriteYamlValue`
- `AppendYamlListItem`

Retain the experimental loader for these Behaviors. Excluding the staged IDs is not a reason to remove the whole plugin.

## Tests

After building and sourcing the package in an isolated workspace with the matching Runtime:

```bash
colcon test --packages-select experimental_behaviors --executor sequential \
  --ctest-args -R 'test_behavior_plugins|test_robot_state_compatibility' --output-on-failure
colcon test-result --verbose
```

`test_behavior_plugins` loads the real plugin and checks all 13 default IDs, the ten context-adapted leaves, each staged ID and all five together in both loader orders, unchanged replacement builders, invalid arrays without factory changes, unconfigured collisions, and read-only parameter enforcement. `test_robot_state_compatibility` checks the vector and Eigen adapter paths with test doubles; it does not replace a build against each supported Runtime.

## License

Contributions are licensed under [BSD-3-Clause](LICENSE), as specified in [CONTRIBUTING.md](CONTRIBUTING.md). Existing first-party source headers are retained.
