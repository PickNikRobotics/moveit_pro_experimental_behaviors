cmake_minimum_required(VERSION 3.22)
project(joint_state_behaviors CXX)

find_package(moveit_pro_package REQUIRED)
moveit_pro_package()

set(THIS_PACKAGE_INCLUDE_DEPENDS
    behaviortree_cpp_picknik
    geometry_msgs
    moveit_msgs
    moveit_pro_base
    moveit_pro_behavior_interface
    pluginlib
    rclcpp
    sensor_msgs
    tl_expected)
foreach(package IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${package} REQUIRED)
endforeach()

find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)

add_library(
  ${PROJECT_NAME} SHARED
  src/compute_joint_state_distance.cpp
  src/create_empty_multi_dof_joint_state.cpp
  src/filter_robot_joint_state.cpp
  src/filter_robot_state_to_joints.cpp
  src/get_joint_value_from_robot_joint_state.cpp
  src/is_joint_position_in_range.cpp
  src/robot_state_from_pose_axis.cpp
  src/set_joint_state_values.cpp
  src/set_joint_value_in_robot_joint_state.cpp
  src/register_joint_state_behaviors.cpp
  src/register_behaviors.cpp)
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

target_link_libraries(${PROJECT_NAME} PRIVATE fmt::fmt)
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
  moveit_pro_behavior_interface joint_state_behaviors_plugin_description.xml)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
