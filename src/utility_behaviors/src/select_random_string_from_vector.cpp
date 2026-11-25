#include "utility_behaviors/select_random_string_from_vector.hpp"

#include <rclcpp/rclcpp.hpp>

#include <random>

namespace utility_behaviors
{
SelectRandomStringFromVector::SelectRandomStringFromVector(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList SelectRandomStringFromVector::providedPorts()
{
  return { BT::InputPort<std::vector<std::string>>("input_vector", "Input vector to sample from"),
           BT::OutputPort<std::string>("random_selection", "Randomly selected item from input_vector") };
}

BT::KeyValueVector SelectRandomStringFromVector::metadata()
{
  return { { "description", "Selects one random element from a vector of strings" }, { "subcategory", "Utilities" } };
}

BT::NodeStatus SelectRandomStringFromVector::tick()
{
  auto input_vector_result = getInput<std::vector<std::string>>("input_vector");
  if (!input_vector_result)
  {
    RCLCPP_ERROR(getBehaviorContext()->node->get_logger(), "Failed to read input port 'input_vector': %s",
                 input_vector_result.error().c_str());
    return BT::NodeStatus::FAILURE;
  }

  const auto& input_vector = input_vector_result.value();
  if (input_vector.empty())
  {
    RCLCPP_ERROR(getBehaviorContext()->node->get_logger(), "Input port 'input_vector' is empty");
    return BT::NodeStatus::FAILURE;
  }

  static thread_local std::mt19937 rng(std::random_device{}());
  std::uniform_int_distribution<std::size_t> dist(0, input_vector.size() - 1);
  const std::size_t selected_index = dist(rng);

  if (!setOutput("random_selection", input_vector[selected_index]))
  {
    RCLCPP_ERROR(getBehaviorContext()->node->get_logger(), "Failed to write output port 'random_selection'");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_DEBUG(getBehaviorContext()->node->get_logger(), "Selected index %zu from input_vector with %zu element(s)",
               selected_index, input_vector.size());

  return BT::NodeStatus::SUCCESS;
}

}  // namespace utility_behaviors
