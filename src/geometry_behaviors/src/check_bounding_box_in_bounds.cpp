#include "geometry_behaviors/check_bounding_box_in_bounds.hpp"

#include <iomanip>
#include <sstream>

namespace geometry_behaviors
{

CheckBoundingBoxInBounds::CheckBoundingBoxInBounds(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList CheckBoundingBoxInBounds::providedPorts()
{
  return { BT::InputPort<geometry_msgs::msg::PoseStamped>("box_center_pose", "Center pose of the bounding box"),
           BT::InputPort<std::vector<double>>("box_dimensions", "Dimensions [x, y, z] of the bounding box in meters"),
           BT::InputPort<std::vector<double>>("min_point", "Minimum boundary point [x, y, z] in world frame"),
           BT::InputPort<std::vector<double>>("max_point", "Maximum boundary point [x, y, z] in world frame"),
           BT::OutputPort<geometry_msgs::msg::PoseStamped>("max_volume_center_pose",
                                                           "Center pose of allowed volume for visualization"),
           BT::OutputPort<std::vector<double>>("max_volume_dimensions",
                                               "Dimensions of allowed volume for visualization") };
}

BT::KeyValueVector CheckBoundingBoxInBounds::metadata()
{
  return { { "description", "Checks if a bounding box is within specified world-frame min/max bounds. Returns FAILURE "
                            "if any dimension exceeds, otherwise returns SUCCESS." } };
}

BT::NodeStatus CheckBoundingBoxInBounds::tick()
{
  // Get input: bounding box center pose
  const auto box_center_pose = getInput<geometry_msgs::msg::PoseStamped>("box_center_pose");
  if (!box_center_pose)
  {
    getBehaviorContext()->logger->publishWarnMessage(name(), "Failed to get required input [box_center_pose]: " +
                                                                 box_center_pose.error());
    return BT::NodeStatus::FAILURE;
  }

  // Get input: bounding box dimensions
  const auto box_dimensions = getInput<std::vector<double>>("box_dimensions");
  if (!box_dimensions)
  {
    getBehaviorContext()->logger->publishWarnMessage(name(), "Failed to get required input [box_dimensions]: " +
                                                                 box_dimensions.error());
    return BT::NodeStatus::FAILURE;
  }

  if (box_dimensions->size() != 3)
  {
    getBehaviorContext()->logger->publishWarnMessage(name(),
                                                     "box_dimensions must contain exactly 3 elements [x, y, z], got " +
                                                         std::to_string(box_dimensions->size()));
    return BT::NodeStatus::FAILURE;
  }

  // Get input: min point
  const auto min_point = getInput<std::vector<double>>("min_point");
  if (!min_point)
  {
    getBehaviorContext()->logger->publishWarnMessage(name(),
                                                     "Failed to get required input [min_point]: " + min_point.error());
    return BT::NodeStatus::FAILURE;
  }

  if (min_point->size() != 3)
  {
    getBehaviorContext()->logger->publishWarnMessage(
        name(), "min_point must contain exactly 3 elements [x, y, z], got " + std::to_string(min_point->size()));
    return BT::NodeStatus::FAILURE;
  }

  // Get input: max point
  const auto max_point = getInput<std::vector<double>>("max_point");
  if (!max_point)
  {
    getBehaviorContext()->logger->publishWarnMessage(name(),
                                                     "Failed to get required input [max_point]: " + max_point.error());
    return BT::NodeStatus::FAILURE;
  }

  if (max_point->size() != 3)
  {
    getBehaviorContext()->logger->publishWarnMessage(
        name(), "max_point must contain exactly 3 elements [x, y, z], got " + std::to_string(max_point->size()));
    return BT::NodeStatus::FAILURE;
  }

  // Calculate max volume center and dimensions for visualization
  geometry_msgs::msg::PoseStamped max_volume_center_pose;
  max_volume_center_pose.header = box_center_pose->header;
  max_volume_center_pose.pose.position.x = (min_point->at(0) + max_point->at(0)) / 2.0;
  max_volume_center_pose.pose.position.y = (min_point->at(1) + max_point->at(1)) / 2.0;
  max_volume_center_pose.pose.position.z = (min_point->at(2) + max_point->at(2)) / 2.0;
  max_volume_center_pose.pose.orientation.w = 1.0;

  std::vector<double> max_volume_dimensions = { max_point->at(0) - min_point->at(0),
                                                max_point->at(1) - min_point->at(1),
                                                max_point->at(2) - min_point->at(2) };

  // Set outputs for visualization
  setOutput("max_volume_center_pose", max_volume_center_pose);
  setOutput("max_volume_dimensions", max_volume_dimensions);

  // Extract values
  const auto& center = box_center_pose->pose.position;
  const double half_x = box_dimensions->at(0) / 2.0;
  const double half_y = box_dimensions->at(1) / 2.0;
  const double half_z = box_dimensions->at(2) / 2.0;

  // Calculate bounding box extents in world frame
  // Note: This assumes the bounding box is axis-aligned with the world frame
  // For a more accurate check with rotated boxes, we'd need to transform all 8 corners
  const double box_min_x = center.x - half_x;
  const double box_max_x = center.x + half_x;
  const double box_min_y = center.y - half_y;
  const double box_max_y = center.y + half_y;
  const double box_min_z = center.z - half_z;
  const double box_max_z = center.z + half_z;

  // Check if bounding box is within bounds
  bool x_in_bounds = (box_min_x >= min_point->at(0)) && (box_max_x <= max_point->at(0));
  bool y_in_bounds = (box_min_y >= min_point->at(1)) && (box_max_y <= max_point->at(1));
  bool z_in_bounds = (box_min_z >= min_point->at(2)) && (box_max_z <= max_point->at(2));

  // Format debug message
  std::ostringstream debug_msg;
  debug_msg << std::fixed << std::setprecision(3);
  debug_msg << "Bounding Box Check: ";
  debug_msg << "Center=(" << center.x << ", " << center.y << ", " << center.z << "), ";
  debug_msg << "Dims=(" << box_dimensions->at(0) << ", " << box_dimensions->at(1) << ", " << box_dimensions->at(2)
            << "), ";
  debug_msg << "Box X=[" << box_min_x << ", " << box_max_x << "], ";
  debug_msg << "Y=[" << box_min_y << ", " << box_max_y << "], ";
  debug_msg << "Z=[" << box_min_z << ", " << box_max_z << "]; ";
  debug_msg << "Bounds X=[" << min_point->at(0) << ", " << max_point->at(0) << "], ";
  debug_msg << "Y=[" << min_point->at(1) << ", " << max_point->at(1) << "], ";
  debug_msg << "Z=[" << min_point->at(2) << ", " << max_point->at(2) << "]";

  if (x_in_bounds && y_in_bounds && z_in_bounds)
  {
    getBehaviorContext()->logger->publishInfoMessage(name() + ": " + debug_msg.str() + " - PASSED");
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    std::ostringstream failure_msg;
    failure_msg << debug_msg.str() << " - FAILED: ";
    if (!x_in_bounds)
      failure_msg << "X out of bounds; ";
    if (!y_in_bounds)
      failure_msg << "Y out of bounds; ";
    if (!z_in_bounds)
      failure_msg << "Z out of bounds";

    getBehaviorContext()->logger->publishWarnMessage(name() + ": " + failure_msg.str());
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace geometry_behaviors
