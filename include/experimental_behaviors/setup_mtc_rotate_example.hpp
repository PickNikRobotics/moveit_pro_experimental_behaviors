// Example behavior split out from setup_mtc_debug_stage.hpp
#pragma once

#include <experimental_behaviors/setup_mtc_debug_stage.hpp>

#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/solvers.h>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <Eigen/Geometry>

namespace moveit_studio::behaviors::examples
{

/**
 * @brief Example MTC behavior using the MTCStageHelper template
 */
class SetupMTCRotateExample final : public MTCStageHelper<SetupMTCRotateExample>
{
  MTC_STAGE_BEHAVIOR_INTERFACE(SetupMTCRotateExample,
                               "Example Rotation",
                               "to perform a cartesian <b>rotation</b> about a specified axis.")

public:
  // Port names for additional inputs specific to rotation
  static constexpr auto kPortIDAxisX = "axis_x";
  static constexpr auto kPortIDAxisY = "axis_y";
  static constexpr auto kPortIDAxisZ = "axis_z";
  static constexpr auto kPortIDDistance = "angular_distance";
  static constexpr auto kPortIDDistanceTolerance = "distance_tolerance";
  static constexpr auto kPortIDHandFrame = "hand_frame";
  static constexpr auto kPortIDAxisFrame = "axis_frame";

  static BT::PortsList providedPorts()
  {
    return MTCCommonPorts::mergePorts({
      BT::InputPort<double>(kPortIDAxisX, 0.0, "X component of rotation axis."),
      BT::InputPort<double>(kPortIDAxisY, 0.0, "Y component of rotation axis."),
      BT::InputPort<double>(kPortIDAxisZ, 1.0, "Z component of rotation axis."),
      BT::InputPort<double>(kPortIDDistance, 0.10, "Distance to rotate around axis (rad)"),
      BT::InputPort<double>(kPortIDDistanceTolerance, 0.01,
                            "Planning will succeed if rotation is more or less than the desired rotation by this amount (rad)"),
      BT::InputPort<std::string>(kPortIDHandFrame, "grasp_link", "Robot link to rotate (IK frame)."),
      BT::InputPort<std::string>(kPortIDAxisFrame, "world", "Reference frame for axis."),
    });
  }

  tl::expected<std::unique_ptr<moveit::task_constructor::Stage>, std::string>
  createStage(const StageInputs& inputs)
  {
    const auto custom_ports = getRequiredInputs(
        getInput<double>(kPortIDAxisX), getInput<double>(kPortIDAxisY), getInput<double>(kPortIDAxisZ),
        getInput<double>(kPortIDDistance), getInput<double>(kPortIDDistanceTolerance),
        getInput<std::string>(kPortIDHandFrame), getInput<std::string>(kPortIDAxisFrame)
    );

    if (!custom_ports.has_value())
    {
      return tl::make_unexpected("Failed to get custom input: " + custom_ports.error());
    }

    const auto& [axis_x, axis_y, axis_z, distance, distance_tolerance, hand_frame, axis_frame] = custom_ports.value();

    // Validate rotation axis
    const Eigen::Vector3d axis{axis_x, axis_y, axis_z};
    if (axis.norm() <= std::numeric_limits<double>::epsilon())
    {
      return tl::make_unexpected("Rotation axis must have a non-zero component.");
    }

    // Validate distances
    if (distance < 0.0 || distance_tolerance < 0.0)
    {
      return tl::make_unexpected("Invalid distance or tolerance (radians).");
    }

    // Validate frames and planning group
    if (auto res = MTCValidation::validateFrame(inputs.task->getRobotModel(), hand_frame, "Hand frame"); !res)
    {
      return tl::make_unexpected(res.error());
    }
    if (auto res = MTCValidation::validateFrame(inputs.task->getRobotModel(), axis_frame, "Axis frame"); !res)
    {
      return tl::make_unexpected(res.error());
    }
    if (auto res = MTCValidation::validatePlanningGroup(inputs.task->getRobotModel(), inputs.planning_group_name); !res)
    {
      return tl::make_unexpected(res.error());
    }

    auto planner = std::make_shared<moveit::task_constructor::solvers::CartesianPath>();

    auto stage = std::make_unique<moveit::task_constructor::stages::MoveRelative>(kDefaultStageName, planner);
    stage->setGroup(inputs.planning_group_name);
    stage->setIKFrame(hand_frame);

    // Angular-only Twist in axis_frame; "distance" interpreted as radians
    geometry_msgs::msg::TwistStamped twist;
    twist.header.frame_id = axis_frame;
    twist.twist.linear.x = twist.twist.linear.y = twist.twist.linear.z = 0.0;
    const Eigen::Vector3d axis_n = axis.normalized();
    twist.twist.angular.x = axis_n.x();
    twist.twist.angular.y = axis_n.y();
    twist.twist.angular.z = axis_n.z();

    stage->setDirection(twist);
    stage->setMinMaxDistance(distance - distance_tolerance, distance + distance_tolerance);

    return stage;
  }
};

}  // namespace moveit_studio::behaviors::examples

