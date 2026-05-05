// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <experimental_behaviors/set_blackboard_by_key.hpp>

#include <moveit_pro_behavior_interface/metadata_fields.hpp>

namespace
{
inline constexpr auto kDescriptionSetBlackboardByKey = R"(
                <p>
                    Writes a source port's value to a blackboard entry whose key is computed at runtime
                    (typically via a Script node building a string from other blackboard variables).
                    The key may be prefixed with '@' to address the root blackboard.
                </p>
                <p>
                    Use this instead of BT.CPP's native SetBlackboard when the source is an Any-typed
                    port (e.g. produced by a Script `:=` assignment): SetBlackboard runs a
                    string-to-destination-type conversion that silently empties the value when the
                    destination type is AnyTypeAllowed and no converter is registered. This behavior
                    copies the source Any directly.
                </p>
            )";

constexpr auto kPortIDKey = "key";
constexpr auto kPortIDValue = "value";
}  // namespace

namespace experimental_behaviors
{
SetBlackboardByKey::SetBlackboardByKey(const std::string& name, const BT::NodeConfiguration& config,
                                       const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList SetBlackboardByKey::providedPorts()
{
  return { BT::InputPort<std::string>(kPortIDKey,
                                      "Blackboard key to write to. May be constructed dynamically via Script. "
                                      "Prefix with '@' for root-scope access."),
           BT::InputPort(kPortIDValue, "Source value. Accepts a blackboard pointer ({some_port}) whose "
                                       "Any contents are copied verbatim, or a literal string.") };
}

BT::KeyValueVector SetBlackboardByKey::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "Blackboard" },
           { moveit_pro::behaviors::kDescriptionMetadataKey, kDescriptionSetBlackboardByKey } };
}

BT::NodeStatus SetBlackboardByKey::tick()
{
  std::string key;
  if (!getInput<std::string>(kPortIDKey, key) || key.empty())
  {
    shared_resources_->logger->publishFailureMessage(name(), "Missing or empty input port [key]");
    return BT::NodeStatus::FAILURE;
  }

  // Read the raw port expression from config rather than going through
  // getInput<std::string>, which would stringify whatever the source port
  // holds. We need the untouched Any so trajectories, vectors, etc. survive
  // the copy.
  const auto value_it = config().input_ports.find(kPortIDValue);
  if (value_it == config().input_ports.end())
  {
    shared_resources_->logger->publishFailureMessage(name(), "Missing input port [value]");
    return BT::NodeStatus::FAILURE;
  }
  const std::string& value_expr = value_it->second;

  BT::StringView stripped_key;
  BT::Any out_value;
  BT::TypeInfo src_info;
  if (BT::TreeNode::isBlackboardPointer(value_expr, &stripped_key))
  {
    const auto input_key = std::string(stripped_key);
    auto src_entry = config().blackboard->getEntry(input_key);
    if (!src_entry)
    {
      shared_resources_->logger->publishFailureMessage(
          name(), "Source port '" + input_key + "' for [value] does not reference an existing blackboard entry.");
      return BT::NodeStatus::FAILURE;
    }
    out_value = src_entry->value;
    src_info = src_entry->info;
  }
  else
  {
    out_value = BT::Any(value_expr);
    src_info = BT::TypeInfo::Create<std::string>();
  }

  // Ensure the destination entry exists before writing. Create it with the
  // source entry's TypeInfo so downstream consumers that inspect entry->info
  // see a consistent type (e.g. AnyTypeAllowed vs std::string vs trajectory).
  auto dst_entry = config().blackboard->getEntry(key);
  if (!dst_entry)
  {
    config().blackboard->createEntry(key, src_info);
    dst_entry = config().blackboard->getEntry(key);
    if (!dst_entry)
    {
      shared_resources_->logger->publishFailureMessage(name(), "Failed to create blackboard entry '" + key + "'");
      return BT::NodeStatus::FAILURE;
    }
  }

  {
    std::scoped_lock lock(dst_entry->entry_mutex);
    dst_entry->value = out_value;
    dst_entry->sequence_id++;
    dst_entry->stamp = std::chrono::steady_clock::now().time_since_epoch();
  }

  return BT::NodeStatus::SUCCESS;
}
}  // namespace experimental_behaviors
