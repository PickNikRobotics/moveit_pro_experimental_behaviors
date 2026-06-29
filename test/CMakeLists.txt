find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(test_behavior_plugins test_behavior_plugins.cpp)
ament_target_dependencies(test_behavior_plugins ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Unit tests for the pure waypoint-selection logic in RetimeJointTrajectory. Links the package
# library for the function definition; no ROS runtime is required.
ament_add_gtest(test_retime_joint_trajectory test_retime_joint_trajectory.cpp)
target_link_libraries(test_retime_joint_trajectory experimental_behaviors)
ament_target_dependencies(test_retime_joint_trajectory ${THIS_PACKAGE_INCLUDE_DEPENDS})
