// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <experimental_behaviors/get_blackboard_by_key.hpp>

#include <moveit_pro_behavior_interface/metadata_fields.hpp>

namespace
{
inline constexpr auto kDescriptionGetBlackboardByKey = R"(
                <p>
                    Reads a blackboard entry whose key is computed at runtime (typically via a Script node
                    building a string from other blackboard variables) and copies the entry's value to an
                    output port. The key may be prefixed with '@' to address the root blackboard.
                </p>
                <p>
                    Returns FAILURE if the key refers to a missing blackboard entry. This is the read-side
                    complement to BT.CPP's native SetBlackboard.
                </p>
            )";

constexpr auto kPortIDKey = "key";
constexpr auto kPortIDValue = "value";
}  // namespace

namespace experimental_behaviors
{
GetBlackboardByKey::GetBlackboardByKey(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList GetBlackboardByKey::providedPorts()
{
  return { BT::InputPort<std::string>(kPortIDKey,
                                      "Blackboard key to read from. May be constructed dynamically via Script. "
                                      "Prefix with '@' for root-scope access."),
           BT::OutputPort(kPortIDValue, "Value read from the blackboard entry referenced by `key`.") };
}

BT::KeyValueVector GetBlackboardByKey::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "Blackboard" },
           { moveit_pro::behaviors::kDescriptionMetadataKey, kDescriptionGetBlackboardByKey } };
}

BT::NodeStatus GetBlackboardByKey::tick()
{
  std::string key;
  if (!getInput<std::string>(kPortIDKey, key) || key.empty())
  {
    shared_resources_->logger->publishFailureMessage(name(), "Missing or empty input port [key]");
    return BT::NodeStatus::FAILURE;
  }

  auto src_entry = config().blackboard->getEntry(key);
  if (!src_entry)
  {
    shared_resources_->logger->publishFailureMessage(
        name(), "Blackboard entry '" + key + "' does not exist (cache miss).");
    return BT::NodeStatus::FAILURE;
  }

  if (src_entry->value.empty())
  {
    shared_resources_->logger->publishFailureMessage(
        name(), "Blackboard entry '" + key + "' exists but holds no value.");
    return BT::NodeStatus::FAILURE;
  }

  // Route through the blackboard directly so the stored BT::Any is copied
  // without forcing a concrete type — the consumer's input port decides the
  // type at read time. Mirrors the pattern of BT.CPP's SetBlackboardNode
  // (see include/behaviortree_cpp/actions/set_blackboard_node.h) in reverse.
  const auto output_remap = config().output_ports.find(kPortIDValue);
  if (output_remap == config().output_ports.end())
  {
    shared_resources_->logger->publishFailureMessage(name(), "Output port [value] is not connected.");
    return BT::NodeStatus::FAILURE;
  }
  std::string dst_key = output_remap->second;
  // Strip leading/trailing braces from blackboard pointer syntax "{name}".
  if (dst_key.size() >= 2 && dst_key.front() == '{' && dst_key.back() == '}')
  {
    dst_key = dst_key.substr(1, dst_key.size() - 2);
  }
  config().blackboard->set(dst_key, src_entry->value);

  return BT::NodeStatus::SUCCESS;
}
}  // namespace experimental_behaviors
