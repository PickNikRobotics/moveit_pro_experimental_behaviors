#include <experimental_behaviors/mtc_debug_stage.hpp>
#include <experimental_behaviors/setup_mtc_debug_stage.hpp>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/stages/passthrough.h>
#include <moveit/task_constructor/solvers.h>
#include <geometry_msgs/msg/twist_stamped.hpp>

namespace moveit_studio::behaviors {

class DebugPassThrough : public moveit::task_constructor::stages::PassThrough {
public:
  DebugPassThrough(const std::string& name, double cost, const std::string& comment)
    : moveit::task_constructor::stages::PassThrough(name), cost_(cost), comment_(comment) {}

protected:
  void onNewSolution(const moveit::task_constructor::SolutionBase& s) override {
    // Add custom cost and comment to each child solution
    liftSolution(s, s.cost() + cost_, comment_);
  }
private:
  double cost_;
  std::string comment_;
};

BT::PortsList SetupMTCDebugStage::providedPorts() {
  return MTCCommonPorts::mergePorts({
    BT::InputPort<double>(kPortIDCustomParam, 1.0, "Stage cost"),
    BT::InputPort<std::string>("ik_frame", "grasp_link", "IK frame (robot link)"),
    BT::InputPort<std::string>("direction_frame", "world", "Frame for direction"),
    BT::InputPort<double>("linear_x", 0.0, "Direction linear x"),
    BT::InputPort<double>("linear_y", 0.0, "Direction linear y"),
    BT::InputPort<double>("linear_z", 0.0, "Direction linear z"),
    BT::InputPort<double>("angular_x", 0.0, "Direction angular x"),
    BT::InputPort<double>("angular_y", 0.0, "Direction angular y"),
    BT::InputPort<double>("angular_z", 1.0, "Direction angular z"),
    BT::InputPort<double>("min_distance", 0.0, "Minimum distance"),
    BT::InputPort<double>("max_distance", 0.1, "Maximum distance"),
    BT::InputPort<std::string>("custom_comment", "", "Custom comment for solutions")
  });
}

tl::expected<std::unique_ptr<moveit::task_constructor::Stage>, std::string> 
SetupMTCDebugStage::createStage(const StageInputs& inputs) {
  // Extract all required inputs
  auto custom_ports = getRequiredInputs(
    getInput<double>(kPortIDCustomParam),
    getInput<std::string>("planning_group_name"),
    getInput<std::string>("ik_frame"),
    getInput<std::string>("direction_frame"),
    getInput<double>("linear_x"), getInput<double>("linear_y"), getInput<double>("linear_z"),
    getInput<double>("angular_x"), getInput<double>("angular_y"), getInput<double>("angular_z"),
    getInput<double>("min_distance"), getInput<double>("max_distance"),
    getInput<std::string>("custom_comment")
  );
  if (!custom_ports.has_value()) {
    return tl::make_unexpected("Failed to get required input: " + custom_ports.error());
  }
  const auto& [stage_cost, planning_group, ik_frame, direction_frame,
    linear_x, linear_y, linear_z, angular_x, angular_y, angular_z,
    min_distance, max_distance, custom_comment] = custom_ports.value();

  // Validate frames and planning group
  if (auto res = MTCValidation::validateFrame(inputs.task->getRobotModel(), ik_frame, "IK frame"); !res) {
    return tl::make_unexpected(res.error());
  }
  if (auto res = MTCValidation::validateFrame(inputs.task->getRobotModel(), direction_frame, "Direction frame"); !res) {
    return tl::make_unexpected(res.error());
  }
  if (auto res = MTCValidation::validatePlanningGroup(inputs.task->getRobotModel(), planning_group); !res) {
    return tl::make_unexpected(res.error());
  }

  auto planner = std::make_shared<moveit::task_constructor::solvers::CartesianPath>();

  auto move_relative_stage = std::make_unique<moveit::task_constructor::stages::MoveRelative>("move_relative", planner);
  move_relative_stage->setGroup(planning_group);
  move_relative_stage->setIKFrame(ik_frame);

  geometry_msgs::msg::TwistStamped twist;
  twist.header.frame_id = direction_frame;
  twist.twist.linear.x = linear_x;
  twist.twist.linear.y = linear_y;
  twist.twist.linear.z = linear_z;
  twist.twist.angular.x = angular_x;
  twist.twist.angular.y = angular_y;
  twist.twist.angular.z = angular_z;
  move_relative_stage->setDirection(twist);
  move_relative_stage->setMinMaxDistance(min_distance, max_distance);

  auto passthrough_stage = std::make_unique<DebugPassThrough>("passthrough", stage_cost, custom_comment);
  passthrough_stage->insert(std::move(move_relative_stage));
  return passthrough_stage;
}

} // namespace moveit_studio::behaviors
