cmake_minimum_required(VERSION 3.22)
project(grasping_behaviors CXX)

find_package(moveit_pro_package REQUIRED)
moveit_pro_package()

set(THIS_PACKAGE_INCLUDE_DEPENDS
    behaviortree_cpp_picknik
    geometry_msgs
    moveit_pro_behavior_interface
    moveit_studio_vision_msgs
    pluginlib
    rclcpp
    sensor_msgs
    shape_msgs
    tf2_eigen
    tl_expected
    visualization_msgs)
foreach(package IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${package} REQUIRED)
endforeach()

find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)

add_library(
  ${PROJECT_NAME} SHARED
  src/determine_pick_alignment.cpp
  src/extract_graspable_object_primitive.cpp
  src/filter_graspable_objects.cpp
  src/generate_grasp_candidates.cpp
  src/grasp_candidate_generation.cpp
  src/register_grasping_behaviors.cpp
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
  moveit_pro_behavior_interface grasping_behaviors_plugin_description.xml)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS} Eigen3)
ament_package()
