find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(test_behavior_plugins test_behavior_plugins.cpp)
target_link_libraries(test_behavior_plugins ${PROJECT_NAME})

ament_add_gtest(test_behavior_index test_behavior_index.cpp)
target_link_libraries(test_behavior_index ${PROJECT_NAME})
target_compile_definitions(
  test_behavior_index PRIVATE
  BEHAVIOR_INDEX_DATA_FILE="${PROJECT_SOURCE_DIR}/docs/behavior_index/behavior_index_data.js")

# Pure mapping from a jog command to a clamped base twist.
ament_add_gtest(test_teleoperate_base_twist_mapping test_teleoperate_base_twist_mapping.cpp)
target_link_libraries(test_teleoperate_base_twist_mapping ${PROJECT_NAME})

# Runs TeleoperateBase over ROS through this folder's registration function.
ament_add_gtest(test_teleoperate_base_ros test_teleoperate_base_ros.cpp)
target_link_libraries(test_teleoperate_base_ros ${PROJECT_NAME})
