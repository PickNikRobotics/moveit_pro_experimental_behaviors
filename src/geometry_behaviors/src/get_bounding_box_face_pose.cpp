#include "geometry_behaviors/get_bounding_box_face_pose.hpp"

#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.hpp>

namespace geometry_behaviors
{

GetBoundingBoxFacePose::GetBoundingBoxFacePose(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList GetBoundingBoxFacePose::providedPorts()
{
  return { BT::InputPort<geometry_msgs::msg::PoseStamped>("box_center_pose", "Center pose of the bounding box."),
           BT::InputPort<std::vector<double>>("box_dimensions",
                                              "Dimensions [length, width, height] of the bounding box in meters."),
           BT::InputPort<std::string>("face", "+x",
                                      "Signed axis selecting the face in the box's local frame. One of "
                                      "\"+x\", \"-x\", \"+y\", \"-y\", \"+z\", \"-z\"."),
           BT::InputPort<double>("offset", 0.0,
                                 "Additional outward distance along the face normal, in meters. "
                                 "Positive = outside the box; negative = toward the box center."),
           BT::OutputPort<geometry_msgs::msg::PoseStamped>(
               "face_pose", "Face-center pose (orientation = box orientation, header copied from box_center_pose)."),
           BT::OutputPort<double>("face_position_x", "Face X coordinate in the input pose's frame."),
           BT::OutputPort<double>("face_position_y", "Face Y coordinate in the input pose's frame."),
           BT::OutputPort<double>("face_position_z", "Face Z coordinate in the input pose's frame.") };
}

BT::KeyValueVector GetBoundingBoxFacePose::metadata()
{
  return { { "description", "Computes a PoseStamped at the center of one face of an oriented bounding box, "
                            "optionally offset outward along the face normal." } };
}

BT::NodeStatus GetBoundingBoxFacePose::tick()
{
  const auto box_center_pose = getInput<geometry_msgs::msg::PoseStamped>("box_center_pose");
  if (!box_center_pose)
  {
    getBehaviorContext()->logger->publishWarnMessage(name(), "Failed to get required input [box_center_pose]: " +
                                                                 box_center_pose.error());
    return BT::NodeStatus::FAILURE;
  }

  const auto box_dimensions = getInput<std::vector<double>>("box_dimensions");
  if (!box_dimensions)
  {
    getBehaviorContext()->logger->publishWarnMessage(name(), "Failed to get required input [box_dimensions]: " +
                                                                 box_dimensions.error());
    return BT::NodeStatus::FAILURE;
  }
  if (box_dimensions->size() != 3)
  {
    getBehaviorContext()->logger->publishWarnMessage(
        name(), "box_dimensions must contain exactly 3 elements [length, width, height], got " +
                    std::to_string(box_dimensions->size()));
    return BT::NodeStatus::FAILURE;
  }

  const auto face = getInput<std::string>("face");
  if (!face)
  {
    getBehaviorContext()->logger->publishWarnMessage(name(), "Failed to get required input [face]: " + face.error());
    return BT::NodeStatus::FAILURE;
  }

  const auto offset = getInput<double>("offset");
  if (!offset)
  {
    getBehaviorContext()->logger->publishWarnMessage(name(), "Failed to get required input [offset]: " + offset.error());
    return BT::NodeStatus::FAILURE;
  }

  // Parse signed-axis token.
  int axis_index = -1;
  double sign = 0.0;
  if (face.value() == "+x")
  {
    axis_index = 0;
    sign = +1.0;
  }
  else if (face.value() == "-x")
  {
    axis_index = 0;
    sign = -1.0;
  }
  else if (face.value() == "+y")
  {
    axis_index = 1;
    sign = +1.0;
  }
  else if (face.value() == "-y")
  {
    axis_index = 1;
    sign = -1.0;
  }
  else if (face.value() == "+z")
  {
    axis_index = 2;
    sign = +1.0;
  }
  else if (face.value() == "-z")
  {
    axis_index = 2;
    sign = -1.0;
  }
  else
  {
    getBehaviorContext()->logger->publishWarnMessage(
        name(), "face must be one of \"+x\", \"-x\", \"+y\", \"-y\", \"+z\", \"-z\"; got \"" + face.value() + "\"");
    return BT::NodeStatus::FAILURE;
  }

  // Distance from box center to face center along the chosen axis, plus the outward offset.
  const double half_extent = box_dimensions->at(axis_index) / 2.0;
  const double signed_distance = sign * (half_extent + offset.value());

  Eigen::Vector3d local_offset = Eigen::Vector3d::Zero();
  local_offset[axis_index] = signed_distance;

  // Rotate the local-frame offset into the pose's frame using the box orientation.
  Eigen::Quaterniond box_q;
  tf2::fromMsg(box_center_pose->pose.orientation, box_q);
  const Eigen::Vector3d world_offset = box_q * local_offset;

  geometry_msgs::msg::PoseStamped face_pose;
  face_pose.header = box_center_pose->header;
  face_pose.pose.orientation = box_center_pose->pose.orientation;
  face_pose.pose.position.x = box_center_pose->pose.position.x + world_offset.x();
  face_pose.pose.position.y = box_center_pose->pose.position.y + world_offset.y();
  face_pose.pose.position.z = box_center_pose->pose.position.z + world_offset.z();

  setOutput("face_pose", face_pose);
  setOutput("face_position_x", face_pose.pose.position.x);
  setOutput("face_position_y", face_pose.pose.position.y);
  setOutput("face_position_z", face_pose.pose.position.z);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace geometry_behaviors
