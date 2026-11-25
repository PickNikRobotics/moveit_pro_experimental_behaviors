cmake_minimum_required(VERSION 3.22)
project(geometry_behaviors CXX)

find_package(moveit_pro_package REQUIRED)
moveit_pro_package()

set(THIS_PACKAGE_INCLUDE_DEPENDS
    behaviortree_cpp_picknik
    geometry_msgs
    moveit_pro_behavior_interface
    pluginlib
    rclcpp
    tf2
    tf2_eigen
    tf2_geometry_msgs
    tf2_ros)
foreach(package IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${package} REQUIRED)
endforeach()

find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)

add_library(
  ${PROJECT_NAME} SHARED
  src/axis_rotated_candidate_generation.cpp
  src/check_bounding_box_in_bounds.cpp
  src/compute_camera_pose_from_tag.cpp
  src/compute_relative_axis_dot.cpp
  src/create_pose_stamped_grid.cpp
  src/express_world_vector_in_link_frame.cpp
  src/generate_axis_rotated_candidates.cpp
  src/get_bounding_box_face_pose.cpp
  src/is_pose_in_box.cpp
  src/orient_pose_axis_to_reference_axis.cpp
  src/orient_pose_plane_to_reference_plane.cpp
  src/register_geometry_behaviors.cpp
  src/register_behaviors.cpp)
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)
target_link_libraries(${PROJECT_NAME} PRIVATE spdlog::spdlog)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY docs DESTINATION share/${PROJECT_NAME})
install(FILES README.md LICENSE NOTICE behavior_plugin.yaml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  moveit_pro_behavior_test(${PROJECT_NAME})
endif()

pluginlib_export_plugin_description_file(
  moveit_pro_behavior_interface geometry_behaviors_plugin_description.xml)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS} Eigen3)
ament_package()
