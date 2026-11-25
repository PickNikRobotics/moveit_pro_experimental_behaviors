find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(test_behavior_plugins test_behavior_plugins.cpp)
target_link_libraries(test_behavior_plugins ${PROJECT_NAME})

ament_add_gtest(test_behavior_index test_behavior_index.cpp)
target_link_libraries(test_behavior_index ${PROJECT_NAME})
target_compile_definitions(
  test_behavior_index PRIVATE
  BEHAVIOR_INDEX_DATA_FILE="${PROJECT_SOURCE_DIR}/docs/behavior_index/behavior_index_data.js")

find_package(ament_cmake_gmock REQUIRED)

# jointPosition is a free function over a JointState, so the test needs no executor.
ament_add_gmock(test_is_joint_position_in_range test_is_joint_position_in_range.cpp TIMEOUT 10)
target_link_libraries(test_is_joint_position_in_range ${PROJECT_NAME})
