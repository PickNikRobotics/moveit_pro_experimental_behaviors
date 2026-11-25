find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(test_behavior_plugins test_behavior_plugins.cpp)
target_link_libraries(test_behavior_plugins ${PROJECT_NAME})

ament_add_gtest(test_behavior_index test_behavior_index.cpp)
target_link_libraries(test_behavior_index ${PROJECT_NAME})
target_compile_definitions(
  test_behavior_index PRIVATE
  BEHAVIOR_INDEX_DATA_FILE="${PROJECT_SOURCE_DIR}/docs/behavior_index/behavior_index_data.js")

# Unit tests for the header-only helpers the YAML Behaviors share. Links the
# library only for its include directory; the helpers need no yaml-cpp.
ament_add_gtest(test_yaml_file_helpers test_yaml_file_helpers.cpp)
target_link_libraries(test_yaml_file_helpers ${PROJECT_NAME})
ament_target_dependencies(test_yaml_file_helpers ament_index_cpp)

# resolveKey, the key lookup of ReadYamlValue and ReadYamlList. Its header is private to src/.
ament_add_gtest(test_yaml_key_lookup test_yaml_key_lookup.cpp)
target_include_directories(test_yaml_key_lookup PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../src)
target_link_libraries(test_yaml_key_lookup ${PROJECT_NAME})
if(TARGET yaml-cpp::yaml-cpp)
  target_link_libraries(test_yaml_key_lookup yaml-cpp::yaml-cpp)
else()
  target_link_libraries(test_yaml_key_lookup yaml-cpp)
endif()
