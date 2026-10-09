find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(test_behavior_plugins test_behavior_plugins.cpp)
ament_target_dependencies(test_behavior_plugins ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Unit tests for the header-only helpers the YAML behaviors share. Links the
# package library for its include directory; the helpers themselves need no
# yaml-cpp, which is why they can live in installed headers.
ament_add_gtest(test_yaml_file_helpers test_yaml_file_helpers.cpp)
target_link_libraries(test_yaml_file_helpers experimental_behaviors)
ament_target_dependencies(test_yaml_file_helpers ament_index_cpp)

# JSON Behaviors. The editing and transport tests load the plugin through
# pluginlib, so they do not link the package library (that would load it twice).
ament_add_gtest(test_json_utils json_behaviors/test_json_utils.cpp)
target_link_libraries(test_json_utils experimental_behaviors)

ament_add_gtest(test_json_editing json_behaviors/test_json_editing.cpp)
ament_target_dependencies(test_json_editing ${THIS_PACKAGE_INCLUDE_DEPENDS})
# json_serialization.hpp (Pro's ROS message converters) parses through yaml-cpp.
if(TARGET yaml-cpp::yaml-cpp)
  target_link_libraries(test_json_editing yaml-cpp::yaml-cpp)
else()
  target_link_libraries(test_json_editing yaml-cpp)
endif()

ament_add_gtest(test_json_transport json_behaviors/test_json_transport.cpp TIMEOUT 120)
ament_target_dependencies(test_json_transport ${THIS_PACKAGE_INCLUDE_DEPENDS})

# REST Behaviors. Loads the plugin through pluginlib, like the JSON transport tests.
ament_add_gtest(test_send_http_request restful_behaviors/test_send_http_request.cpp TIMEOUT 120)
ament_target_dependencies(test_send_http_request ${THIS_PACKAGE_INCLUDE_DEPENDS})
