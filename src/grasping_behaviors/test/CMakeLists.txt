find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(test_behavior_plugins test_behavior_plugins.cpp)
target_link_libraries(test_behavior_plugins ${PROJECT_NAME})

ament_add_gtest(test_behavior_index test_behavior_index.cpp)
target_link_libraries(test_behavior_index ${PROJECT_NAME})
target_compile_definitions(
  test_behavior_index PRIVATE
  BEHAVIOR_INDEX_DATA_FILE="${PROJECT_SOURCE_DIR}/docs/behavior_index/behavior_index_data.js")

# generatePlaceCandidates is a pure function over Eigen poses, so the test needs no executor.
ament_add_gtest(test_generate_place_candidates test_generate_place_candidates.cpp TIMEOUT 10)
target_link_libraries(test_generate_place_candidates ${PROJECT_NAME})
