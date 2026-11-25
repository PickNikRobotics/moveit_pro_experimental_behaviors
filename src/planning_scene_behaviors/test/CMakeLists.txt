find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(test_behavior_plugins test_behavior_plugins.cpp)
target_link_libraries(test_behavior_plugins ${PROJECT_NAME})

ament_add_gtest(test_behavior_index test_behavior_index.cpp)
target_link_libraries(test_behavior_index ${PROJECT_NAME})
target_compile_definitions(
  test_behavior_index PRIVATE
  BEHAVIOR_INDEX_DATA_FILE="${PROJECT_SOURCE_DIR}/docs/behavior_index/behavior_index_data.js")

find_package(ament_cmake_gmock REQUIRED)

# AddColoredVirtualObjectToPlanningScene::buildRequest is a pure static helper, so the test needs no executor.
ament_add_gmock(test_add_colored_virtual_object_to_planning_scene
                test_add_colored_virtual_object_to_planning_scene.cpp TIMEOUT 10)
target_link_libraries(test_add_colored_virtual_object_to_planning_scene ${PROJECT_NAME})

# GenerateShapeUrdf per-shape output and failure modes, through this folder's registration function.
ament_add_gtest(test_generate_shape_urdf test_generate_shape_urdf.cpp)
target_link_libraries(test_generate_shape_urdf ${PROJECT_NAME})
