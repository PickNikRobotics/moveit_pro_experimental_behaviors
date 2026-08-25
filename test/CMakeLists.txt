find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(test_behavior_plugins test_behavior_plugins.cpp)
ament_target_dependencies(test_behavior_plugins ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Unit tests for the header-only helpers the YAML behaviors share. Links the
# package library for its include directory; the helpers themselves need no
# yaml-cpp, which is why they can live in installed headers.
ament_add_gtest(test_yaml_file_helpers test_yaml_file_helpers.cpp)
target_link_libraries(test_yaml_file_helpers experimental_behaviors)
ament_target_dependencies(test_yaml_file_helpers ament_index_cpp)
