find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(test_behavior_plugins test_behavior_plugins.cpp)
target_link_libraries(test_behavior_plugins ${PROJECT_NAME})

ament_add_gtest(test_behavior_index test_behavior_index.cpp)
target_link_libraries(test_behavior_index ${PROJECT_NAME})
target_compile_definitions(
  test_behavior_index PRIVATE
  BEHAVIOR_INDEX_DATA_FILE="${PROJECT_SOURCE_DIR}/docs/behavior_index/behavior_index_data.js")

# CropPointCloudToCameraFrustum on a small cloud in the camera frame, so no transform is needed.
ament_add_gtest(test_crop_point_cloud_to_camera_frustum test_crop_point_cloud_to_camera_frustum.cpp)
target_link_libraries(test_crop_point_cloud_to_camera_frustum ${PROJECT_NAME})

# findNewestPcdFile and a full load, on files written to a temporary directory.
ament_add_gtest(test_load_latest_point_cloud_from_file test_load_latest_point_cloud_from_file.cpp)
target_link_libraries(test_load_latest_point_cloud_from_file ${PROJECT_NAME})
