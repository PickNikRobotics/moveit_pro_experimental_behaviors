cmake_minimum_required(VERSION 3.22)
project(experimental_behaviors CXX)

find_package(moveit_pro_package REQUIRED)
moveit_pro_package()

# MoveIt Pro links the behaviortree_cpp_picknik fork on Jazzy, while stock
# BehaviorTree.CPP is kept for Nav2 at runtime only and its development files
# are excluded from the image (MoveIt Pro issue #20928). Depend on the fork on
# Jazzy and keep stock behaviortree_cpp on Humble.
if("$ENV{ROS_DISTRO}" STREQUAL "jazzy")
  set(BEHAVIORTREE_CPP_PACKAGE behaviortree_cpp_picknik)
  add_compile_definitions(MOVEIT_PRO_REQUIRE_BTCPP_PICKNIK_FORK)
else()
  set(BEHAVIORTREE_CPP_PACKAGE behaviortree_cpp)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS ament_index_cpp control_msgs geometry_msgs moveit_pro_behavior_interface pluginlib ${BEHAVIORTREE_CPP_PACKAGE} tl_expected trajectory_msgs moveit_pro_base)
foreach(package IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${package} REQUIRED)
endforeach()
find_package(yaml-cpp REQUIRED)
find_package(CURL REQUIRED)

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
  src/get_joint_limits.cpp
  src/set_blackboard_by_key.cpp
  src/trajectory_to_path.cpp
  src/write_yaml_value.cpp
  src/json_behaviors/create_json.cpp
  src/json_behaviors/get_json_field.cpp
  src/json_behaviors/has_json_field.cpp
  src/json_behaviors/json_utils.cpp
  src/json_behaviors/merge_json.cpp
  src/json_behaviors/receive_json_udp.cpp
  src/json_behaviors/remove_json_field.cpp
  src/json_behaviors/send_json_http.cpp
  src/json_behaviors/send_json_udp.cpp
  src/json_behaviors/set_json_field.cpp
  src/json_behaviors/udp_socket.cpp
  src/restful_behaviors/send_http_request.cpp
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
# libcurl backs SendJsonHttp. Like yaml-cpp, no installed header includes it.
target_link_libraries(experimental_behaviors PRIVATE CURL::libcurl)
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
