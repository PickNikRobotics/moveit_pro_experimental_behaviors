#pragma once

#include <behaviortree_cpp/action_node.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

namespace yaml_behaviors
{
/**
 * @brief Reads a value from a YAML file given a package name, relative file path, and nested keys.
 * @details This behavior uses BT::SyncActionNode wrapped in SharedResourcesNode to read a YAML file
 * and extract a string value from a nested key structure.
 */
class ReadYamlValue : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  /**
   * @brief Constructor for ReadYamlValue behavior.
   * @param name The name of a particular instance of this Behavior.
   * @param config Runtime configuration for this Behavior, including port mappings.
   * @param shared_resources Shared resources provided by the behavior context.
   */
  ReadYamlValue(const std::string& name, const BT::NodeConfiguration& config,
                const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  /**
   * @brief Implementation of the required providedPorts() function.
   * @details Inputs: package_name and relative_file_path name the YAML file; key1 (mandatory) to key6
   * (optional) give the key path; case_sensitive matches keys exactly by default.
   *
   * - Input: optional (bool) - When true, an absent key gives SUCCESS with an empty `value`; every other
   *   lookup error still fails. See docs/AGENT_COMMENTS.md.
   * - Output: value (string) - The extracted string value from the YAML file
   * @return A BT::PortsList containing the behavior's ports.
   */
  static BT::PortsList providedPorts();

  /**
   * @brief Implementation of the metadata() function for displaying behavior information.
   * @return A BT::KeyValueVector containing the behavior's metadata.
   */
  static BT::KeyValueVector metadata();

  /**
   * @brief Implementation of BT::SyncActionNode::tick().
   * @details Reads the YAML file, traverses the nested keys, and extracts the value.
   * This function completes synchronously.
   * @return BT::NodeStatus::SUCCESS if the value was read, or if `optional` is set and the
   * key is absent (in which case `value` is empty). FAILURE otherwise.
   */
  BT::NodeStatus tick() override;
};
}  // namespace yaml_behaviors
