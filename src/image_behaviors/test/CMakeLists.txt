find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(test_behavior_plugins test_behavior_plugins.cpp)
target_link_libraries(test_behavior_plugins ${PROJECT_NAME})

ament_add_gtest(test_behavior_index test_behavior_index.cpp)
target_link_libraries(test_behavior_index ${PROJECT_NAME})
target_compile_definitions(
  test_behavior_index PRIVATE
  BEHAVIOR_INDEX_DATA_FILE="${PROJECT_SOURCE_DIR}/docs/behavior_index/behavior_index_data.js")

# Ray and plane math, and IntersectPixelRaysWithPlane with the camera and the plane in one frame.
# pixel_ray.hpp is private to src/.
ament_add_gtest(test_intersect_pixel_rays_with_plane test_intersect_pixel_rays_with_plane.cpp)
target_include_directories(test_intersect_pixel_rays_with_plane PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../src)
target_link_libraries(test_intersect_pixel_rays_with_plane ${PROJECT_NAME})
