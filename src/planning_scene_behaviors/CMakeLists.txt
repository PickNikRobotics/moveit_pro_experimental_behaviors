cmake_minimum_required(VERSION 3.22)
project(planning_scene_behaviors CXX)

find_package(moveit_pro_package REQUIRED)
moveit_pro_package()

set(THIS_PACKAGE_INCLUDE_DEPENDS
    behaviortree_cpp_picknik
    geometry_msgs
    moveit_msgs
    moveit_pro_base
    moveit_pro_behavior_interface
    octomap_msgs
    pluginlib
    rclcpp
    sensor_msgs
    std_msgs
    tl_expected)
foreach(package IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${package} REQUIRED)
endforeach()

find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)
find_package(octomap REQUIRED)

add_library(
  ${PROJECT_NAME} SHARED
  src/add_colored_virtual_object_to_planning_scene.cpp
  src/add_robot_joint_state_to_planning_scene_msg.cpp
  src/apply_planning_scene_diff.cpp
  src/generate_shape_urdf.cpp
  src/get_attached_collision_object_pose_stamped.cpp
  src/get_planning_scene_acm_only.cpp
  src/inflate_octomap.cpp
  src/is_joint_state_in_collision.cpp
  src/register_planning_scene_behaviors.cpp
  src/register_behaviors.cpp)
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

target_link_libraries(${PROJECT_NAME} PRIVATE Eigen3::Eigen)
target_link_libraries(${PROJECT_NAME} PRIVATE fmt::fmt)
target_link_libraries(${PROJECT_NAME} PRIVATE spdlog::spdlog)
target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${OCTOMAP_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${OCTOMAP_LIBRARIES})

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
  moveit_pro_behavior_interface planning_scene_behaviors_plugin_description.xml)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
