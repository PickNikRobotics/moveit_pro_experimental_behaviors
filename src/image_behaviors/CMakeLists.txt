cmake_minimum_required(VERSION 3.22)
project(image_behaviors CXX)

find_package(moveit_pro_package REQUIRED)
moveit_pro_package()

set(THIS_PACKAGE_INCLUDE_DEPENDS
    behaviortree_cpp_picknik
    cv_bridge
    geometry_msgs
    moveit_pro_behavior_interface
    moveit_pro_common
    moveit_studio_vision_msgs
    pluginlib
    rclcpp
    sensor_msgs)
foreach(package IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${package} REQUIRED)
endforeach()

find_package(spdlog REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)

add_library(
  ${PROJECT_NAME} SHARED
  src/find_mask_index_for_point.cpp
  src/find_points_in_masks.cpp
  src/get_highest_confidence_mask.cpp
  src/mark_points_on_image.cpp
  src/save_depth_image_to_file.cpp
  src/register_image_behaviors.cpp
  src/register_behaviors.cpp)
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

target_link_libraries(${PROJECT_NAME} PRIVATE spdlog::spdlog)
target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBRARIES})

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
  moveit_pro_behavior_interface image_behaviors_plugin_description.xml)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
