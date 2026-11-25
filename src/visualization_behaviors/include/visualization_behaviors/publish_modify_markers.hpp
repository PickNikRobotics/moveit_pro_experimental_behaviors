#pragma once

#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

namespace visualization_behaviors
{
/**
 * @brief Publishes or modifies visualization markers on the /visual_markers topic.
 * @details This behavior reads marker parameters from input ports and publishes them to /visual_markers.
 * Markers with the same namespace can be modified or deleted by calling this behavior again with updated parameters
 * and operation type.
 */
class PublishModifyMarkers : public BT::SyncActionNode
{
public:
  /**
   * @brief Constructor for the PublishModifyMarkers behavior.
   * @param name The name of a particular instance of this Behavior. This will be set by the behavior tree factory when
   * this Behavior is created within a new behavior tree.
   * @param config This contains runtime configuration info for this Behavior, such as the mapping between the
   * Behavior's data ports on the behavior tree's blackboard. This will be set by the behavior tree factory when this
   * Behavior is created within a new behavior tree.
   * @param shared_resources Provides access to shared resources like ROS node for publishing.
   * @details An important limitation is that the members of the base Behavior class are not instantiated until after
   * the initialize() function is called, so these classes should not be used within the constructor.
   */
  PublishModifyMarkers(const std::string& name, const BT::NodeConfiguration& config,
                       const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  /**
   * @brief Implementation of the required providedPorts() function for the PublishModifyMarkers Behavior.
   * @details The BehaviorTree.CPP library requires that Behaviors must implement a static function named
   * providedPorts() which defines their input and output ports. If the Behavior does not use any ports, this function
   * must return an empty BT::PortsList. This function returns a list of ports with their names and port info, which is
   * used internally by the behavior tree.
   * @return PortsList containing input ports for namespace, poses, scale, color, marker type, and operation type.
   */
  static BT::PortsList providedPorts();

  /**
   * @brief Implementation of the metadata() function for displaying metadata, such as Behavior description and
   * subcategory, in the MoveIt Studio Developer Tool.
   * @return A BT::KeyValueVector containing the Behavior metadata.
   */
  static BT::KeyValueVector metadata();

  /**
   * @brief Implementation of BT::SyncActionNode::tick() for PublishModifyMarkers.
   * @details This function reads the input ports, creates visualization markers based on the provided parameters,
   * and publishes them to the /visual_markers topic. Markers with the same namespace will replace or delete
   * previously published markers with the same namespace and ID depending on the selected operation type.
   */
  BT::NodeStatus tick() override;

private:
  std::shared_ptr<moveit_pro::behaviors::BehaviorContext> shared_resources_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
};
}  // namespace visualization_behaviors
