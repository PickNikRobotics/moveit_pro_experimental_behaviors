cmake_minimum_required(VERSION 3.22)
project(experimental_behaviors CXX)

find_package(moveit_pro_package REQUIRED)
moveit_pro_package()

set(THIS_PACKAGE_INCLUDE_DEPENDS ament_index_cpp control_msgs geometry_msgs moveit_pro_behavior_interface pluginlib behaviortree_cpp tl_expected trajectory_msgs moveit_pro_base)
foreach(package IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${package} REQUIRED)
endforeach()
find_package(yaml-cpp REQUIRED)

add_library(
  experimental_behaviors
  SHARED
  src/append_yaml_list_item.cpp
  src/create_interface_value.cpp
  src/create_dynamic_interface_group_values.cpp
  src/get_dynamic_interface_group_values.cpp
  src/get_interface_value_from_group.cpp
  src/access_interface_value_from_group.cpp
  src/get_pose_stamped_from_topic.cpp
  src/publish_dynamic_interface_group_values.cpp
  src/get_blackboard_by_key.cpp
  src/set_blackboard_by_key.cpp
  src/trajectory_to_path.cpp
  src/write_yaml_value.cpp
  src/register_behaviors.cpp)
target_include_directories(
  experimental_behaviors
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
# yaml-cpp is an implementation detail: no installed header includes it, so it
# must not reach INTERFACE_LINK_LIBRARIES of the exported target (it is not in
# ament_export_dependencies, so a consumer could not resolve it). PRIVATE here
# forces the keyword signature on ament_target_dependencies too, because CMake
# rejects mixing the plain and keyword forms on one target.
#
# yaml-cpp::yaml-cpp is the target the modern config exports; the bare name is
# the fallback for installs whose config predates it.
if(TARGET yaml-cpp::yaml-cpp)
  target_link_libraries(experimental_behaviors PRIVATE yaml-cpp::yaml-cpp)
else()
  target_link_libraries(experimental_behaviors PRIVATE yaml-cpp)
endif()
ament_target_dependencies(experimental_behaviors PUBLIC
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Install Libraries
install(
  TARGETS experimental_behaviors
  EXPORT experimental_behaviorsTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)

if(BUILD_TESTING)
  moveit_pro_behavior_test(experimental_behaviors)
endif()

# Export the behavior plugins defined in this package so they are available to
# plugin loaders that load the behavior base class library from the
# moveit_pro_behavior package.
pluginlib_export_plugin_description_file(
  moveit_pro_behavior_interface experimental_behaviors_plugin_description.xml)

ament_export_targets(experimental_behaviorsTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
