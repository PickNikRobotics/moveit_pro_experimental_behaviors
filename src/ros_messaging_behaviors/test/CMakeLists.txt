find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(test_behavior_plugins test_behavior_plugins.cpp)
target_link_libraries(test_behavior_plugins ${PROJECT_NAME})

ament_add_gtest(test_behavior_index test_behavior_index.cpp)
target_link_libraries(test_behavior_index ${PROJECT_NAME})
target_compile_definitions(
  test_behavior_index PRIVATE
  BEHAVIOR_INDEX_DATA_FILE="${PROJECT_SOURCE_DIR}/docs/behavior_index/behavior_index_data.js")

# PublishPoseStamped over ROS: the pose arrives unchanged and is latched for a late subscriber.
ament_add_gtest(test_publish_pose_stamped test_publish_pose_stamped.cpp)
target_link_libraries(test_publish_pose_stamped ${PROJECT_NAME})
