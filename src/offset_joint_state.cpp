#include "experimental_behaviors/offset_joint_state.hpp"

#include <spdlog/spdlog.h>
#include <moveit_studio_agent_msgs/msg/robot_joint_state.hpp>
#include <moveit_studio_behavior_interface/get_required_ports.hpp>
#include <moveit_studio_behavior_interface/metadata_fields.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <set>

namespace
{
inline constexpr auto kDescriptionOffsetJointState = R"(
    <p>
        Offsets the positions of specified joints in a moveit_studio_agent_msgs/RobotJointState message.
    </p>
    <p>
        Takes an input joint state message, a vector of joint names, and a vector of position deltas.
        The positions of the specified joints are incremented by the corresponding deltas.
    </p>
    <p>
        The sizes of `joint_names` and `position_deltas` must match.
    </p>
)";

constexpr auto kPortIDJointStateMsg = "joint_state_msg";
constexpr auto kPortIDJointNames = "joint_names";
constexpr auto kPortIDPositionDeltas = "position_deltas";
constexpr auto kPortIDOffsetJointStateMsg = "offset_joint_state_msg";

// Check the inputs and return an error string if the input is invalid, or nothing on success.
tl::expected<void, std::string> checkInputs(const std::vector<std::string>& names, const std::vector<double>& deltas,
                                            const sensor_msgs::msg::JointState& joint_state)
{
  // Check names and deltas
  if (names.empty())
  {
    spdlog::warn("No joint names specified in the input port.");
  }
  if (deltas.empty())
  {
    spdlog::warn("No position deltas specified in the input port.");
  }
  if (names.size() != deltas.size())
  {
    return tl::make_unexpected("Sizes of joint_names and position_deltas must match.");
  }
  std::set<std::string> unique_names(names.begin(), names.end());
  if (unique_names.size() != names.size())
  {
    return tl::make_unexpected("Found a duplicate joint name. Joint names must be unique.");
  }

  // Check joint state
  if (joint_state.name.empty())
  {
    return tl::make_unexpected("Input JointState doesn't contain joint names.");
  }
  if (joint_state.name.size() != joint_state.position.size())
  {
    return tl::make_unexpected("Input JointState sizes of 'position' and 'name' don't match.");
  }

  // Check if all joint names are present in the input joint state
  for (const auto& name : names)
  {
    if (std::find(joint_state.name.begin(), joint_state.name.end(), name) == joint_state.name.end())
    {
      return tl::make_unexpected("Joint name '" + name + "' not found in input joint state.");
    }
  }
  return {};
}

}  // namespace

namespace experimental_behaviors
{
OffsetJointState::OffsetJointState(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config)
{
}

BT::PortsList OffsetJointState::providedPorts()
{
  return { BT::InputPort<moveit_studio_agent_msgs::msg::RobotJointState>(kPortIDJointStateMsg,
                                                                         "Input joint state message"),
           BT::InputPort<std::vector<std::string>>(kPortIDJointNames, "Joint names to offset"),
           BT::InputPort<std::vector<double>>(kPortIDPositionDeltas, "Position deltas for the joints"),
           BT::OutputPort<moveit_studio_agent_msgs::msg::RobotJointState>(kPortIDOffsetJointStateMsg,
                                                                          "Resulting joint state message") };
}

BT::NodeStatus OffsetJointState::tick()
{
  // Get required inputs
  const auto ports = moveit_studio::behaviors::getRequiredInputs(
      getInput<moveit_studio_agent_msgs::msg::RobotJointState>(kPortIDJointStateMsg),
      getInput<std::vector<std::string>>(kPortIDJointNames), getInput<std::vector<double>>(kPortIDPositionDeltas));

  if (!ports.has_value())
  {
    spdlog::error("Failed to get required values from input data port: {}", ports.error());
    return BT::NodeStatus::FAILURE;
  }

  const auto& [input_msg, names, deltas] = ports.value();

  // Validate inputs
  if (auto maybe_ok = checkInputs(names, deltas, input_msg.joint_state); !maybe_ok.has_value())
  {
    spdlog::error("Input validation failed: {}", maybe_ok.error());
    return BT::NodeStatus::FAILURE;
  }

  // Copy input message to output
  auto output_msg = input_msg;

  // Offset the positions
  for (std::size_t i = 0; i < names.size(); ++i)
  {
    const std::string& joint_name = names[i];
    double delta = deltas[i];
    auto it = std::find(output_msg.joint_state.name.begin(), output_msg.joint_state.name.end(), joint_name);
    std::size_t idx = std::distance(output_msg.joint_state.name.begin(), it);
    output_msg.joint_state.position[idx] += delta;
  }

  // Set output
  setOutput(kPortIDOffsetJointStateMsg, output_msg);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace experimental_behaviors
