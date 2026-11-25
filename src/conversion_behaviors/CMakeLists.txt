cmake_minimum_required(VERSION 3.22)
project(conversion_behaviors CXX)

find_package(moveit_pro_package REQUIRED)
moveit_pro_package()

set(THIS_PACKAGE_INCLUDE_DEPENDS
    behaviortree_cpp_picknik
    moveit_pro_behavior_interface
    pluginlib
    rclcpp)
foreach(package IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${package} REQUIRED)
endforeach()

find_package(spdlog REQUIRED)

add_library(
  ${PROJECT_NAME} SHARED
  src/create_vector_of_doubles.cpp
  src/double_to_string.cpp
  src/string_to_double.cpp
  src/register_conversion_behaviors.cpp
  src/register_behaviors.cpp)
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

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
  moveit_pro_behavior_interface conversion_behaviors_plugin_description.xml)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
