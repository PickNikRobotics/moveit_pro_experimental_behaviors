cmake_minimum_required(VERSION 3.22)
project(point_cloud_behaviors CXX)

find_package(moveit_pro_package REQUIRED)
moveit_pro_package()

# PCL reaches VTK, which needs the C MPI target. See docs/AGENT_COMMENTS.md.
if(NOT TARGET MPI::MPI_C)
  enable_language(C)
  find_package(MPI REQUIRED COMPONENTS C)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
    behaviortree_cpp_picknik
    geometry_msgs
    moveit_pro_behavior_interface
    moveit_pro_common
    moveit_studio_vision
    moveit_studio_vision_msgs
    pcl_conversions
    pluginlib
    rclcpp
    sensor_msgs
    tf2_eigen
    tf2_ros
    tl_expected)
foreach(package IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${package} REQUIRED)
endforeach()

find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(PCL REQUIRED COMPONENTS common features filters io kdtree search surface)

add_library(
  ${PROJECT_NAME} SHARED
  src/camera_frustum.cpp
  src/clear_point_cloud_timestamp.cpp
  src/count_points_in_cloud.cpp
  src/crop_point_cloud_to_camera_frustum.cpp
  src/exclude_mask_3d_from_pointcloud.cpp
  src/get_closest_point_pose.cpp
  src/load_latest_point_cloud_from_file.cpp
  src/mirror_pointcloud.cpp
  src/pcl_nn_filter.cpp
  src/poisson_reconstruct.cpp
  src/reconstruct_surface.cpp
  src/register_point_cloud_behaviors.cpp
  src/register_behaviors.cpp)
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

target_link_libraries(${PROJECT_NAME} PRIVATE Eigen3::Eigen)
target_link_libraries(${PROJECT_NAME} PRIVATE spdlog::spdlog)
target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(
  ${PROJECT_NAME} PRIVATE ${PCL_COMMON_LIBRARIES} ${PCL_FEATURES_LIBRARIES}
  ${PCL_FILTERS_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_KDTREE_LIBRARIES}
  ${PCL_SEARCH_LIBRARIES} ${PCL_SURFACE_LIBRARIES})

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
  moveit_pro_behavior_interface point_cloud_behaviors_plugin_description.xml)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
