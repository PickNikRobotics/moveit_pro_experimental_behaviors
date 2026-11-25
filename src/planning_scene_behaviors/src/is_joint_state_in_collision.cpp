#include "planning_scene_behaviors/is_joint_state_in_collision.hpp"

#include <behaviortree_cpp/basic_types.h>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

#include <spdlog/spdlog.h>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>

#include <Eigen/Core>

#include <iomanip>
#include <sstream>

#include <moveit_msgs/msg/planning_scene.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <moveit_pro_base/collision_detection/collision_common.hpp>
#include <moveit_pro_base/planning_scene/planning_scene.hpp>

namespace
{
constexpr auto kPortIDJointState = "joint_state";
constexpr auto kPortIDEoatJointNames = "eoat_joint_names";
constexpr auto kPortIDEoatJointPositions = "eoat_joint_positions";
constexpr auto kPortIDPlanningSceneMsg = "planning_scene_msg";
constexpr auto kPortIDCollidingContacts = "colliding_contacts";

// Maximum number of link/object contact pairs to report for diagnostics.
constexpr std::size_t kMaxContactsToReport = 100;
}  // namespace

namespace planning_scene_behaviors
{

IsJointStateInCollision::IsJointStateInCollision(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList IsJointStateInCollision::providedPorts()
{
  return BT::PortsList(
      { BT::InputPort<sensor_msgs::msg::JointState>(kPortIDJointState, "{joint_state}", "The arm joint state to check."),
        BT::InputPort<std::vector<std::string>>(kPortIDEoatJointNames, "{eoat_joint_names}",
                                                "Names of the EOAT joints to set before checking."),
        BT::InputPort<std::vector<double>>(kPortIDEoatJointPositions, "{eoat_joint_positions}",
                                           "Positions of the EOAT joints to set before checking, same order as "
                                           "eoat_joint_names."),
        BT::InputPort<moveit_msgs::msg::PlanningScene>(kPortIDPlanningSceneMsg, "{planning_scene_msg}",
                                                       "The planning scene providing the world and allowed collision "
                                                       "matrix. Its robot state is overwritten "
                                                       "by joint_state and the EOAT joint values before checking."),
        BT::OutputPort<std::vector<std::string>>(kPortIDCollidingContacts, "{colliding_contacts}",
                                                 "Human-readable 'nameA <-> nameB' pairs for each colliding "
                                                 "link/object pair.") });
}

BT::KeyValueVector IsJointStateInCollision::metadata()
{
  return { { "description", "Fails if an arm joint state plus a candidate EOAT joint configuration is in collision "
                            "within a given planning scene; succeeds otherwise." },
           { "subcategory", "Utility" } };
}

BT::NodeStatus IsJointStateInCollision::tick()
{
  const auto ports =
      moveit_pro::behaviors::getRequiredInputs(getInput<sensor_msgs::msg::JointState>(kPortIDJointState),
                                               getInput<std::vector<std::string>>(kPortIDEoatJointNames),
                                               getInput<std::vector<double>>(kPortIDEoatJointPositions),
                                               getInput<moveit_msgs::msg::PlanningScene>(kPortIDPlanningSceneMsg));

  if (!ports.has_value())
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "Failed to get required value from input data port: " +
                                                                    ports.error());
    return BT::NodeStatus::FAILURE;
  }

  const auto& [joint_state, eoat_joint_names, eoat_joint_positions, planning_scene_msg] = ports.value();

  if (joint_state.name.size() != joint_state.position.size())
  {
    getBehaviorContext()->logger->publishFailureMessage(
        name(), "joint_state.name and joint_state.position must be the same size.");
    return BT::NodeStatus::FAILURE;
  }

  if (eoat_joint_names.size() != eoat_joint_positions.size())
  {
    getBehaviorContext()->logger->publishFailureMessage(
        name(), "eoat_joint_names and eoat_joint_positions must be the same size.");
    return BT::NodeStatus::FAILURE;
  }

  const auto& robot_model = getBehaviorContext()->robot_model;
  if (robot_model == nullptr)
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "Robot model is not available in shared resources.");
    return BT::NodeStatus::FAILURE;
  }

  auto planning_scene = std::make_shared<moveit_pro::base::planning_scene::PlanningScene>(robot_model);
  planning_scene->setPlanningSceneMsg(planning_scene_msg);

  auto& state = planning_scene->getCurrentStateNonConst();
  try
  {
    for (std::size_t i = 0; i < joint_state.name.size(); ++i)
    {
      state.setVariablePosition(joint_state.name[i], joint_state.position[i]);
    }
    for (std::size_t i = 0; i < eoat_joint_names.size(); ++i)
    {
      state.setVariablePosition(eoat_joint_names[i], eoat_joint_positions[i]);
    }
  }
  catch (const std::exception& e)
  {
    getBehaviorContext()->logger->publishFailureMessage(
        name(), "Failed to set joint state on planning scene robot state: " + std::string(e.what()));
    return BT::NodeStatus::FAILURE;
  }
  state.update();

  moveit_pro::base::collision_detection::CollisionRequest request;
  request.contacts = true;
  request.max_contacts = kMaxContactsToReport;
  moveit_pro::base::collision_detection::CollisionResult result;
  planning_scene->checkCollision(request, result);

  std::vector<std::string> colliding_contacts;
  colliding_contacts.reserve(result.contacts.size());
  for (const auto& contact_pair : result.contacts)
  {
    colliding_contacts.push_back(contact_pair.first.first + " <-> " + contact_pair.first.second);
  }
  setOutput(kPortIDCollidingContacts, colliding_contacts);

  if (result.collision)
  {
    std::string contacts_str = colliding_contacts.empty() ? "none reported" : colliding_contacts.front();
    for (std::size_t i = 1; i < colliding_contacts.size(); ++i)
    {
      contacts_str += ", " + colliding_contacts[i];
    }

    std::string eoat_joint_states_str;
    for (std::size_t i = 0; i < eoat_joint_names.size(); ++i)
    {
      if (i > 0)
      {
        eoat_joint_states_str += ", ";
      }
      std::ostringstream position_stream;
      position_stream << std::fixed << std::setprecision(3) << eoat_joint_positions[i];
      eoat_joint_states_str += eoat_joint_names[i] + "=" + position_stream.str();
    }

    getBehaviorContext()->logger->publishWarnMessage(name(), "Joint state is in collision with EOAT joint states [" +
                                                                 eoat_joint_states_str + "]: " + contacts_str);
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace planning_scene_behaviors
