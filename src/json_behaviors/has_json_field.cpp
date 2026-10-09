// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <experimental_behaviors/json_behaviors/has_json_field.hpp>

#include <experimental_behaviors/json_behaviors/json_utils.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

namespace
{
inline constexpr auto kDescriptionHasJsonField = R"(
                <p>
                    Checks whether a JSON document has a field at a JSON Pointer path (RFC 6901). Returns
                    SUCCESS if the field exists, also when its value is null, and FAILURE if it does not.
                </p>
                <p>
                    Use it in a Fallback or a Sequence to branch on an optional field before reading it
                    with <code>GetJsonField</code>. Invalid JSON or an invalid path also return FAILURE,
                    and those cases log a message.
                </p>
            )";

constexpr auto kPortIDJson = "json";
constexpr auto kPortIDPath = "path";
}  // namespace

namespace experimental_behaviors
{
HasJsonField::HasJsonField(const std::string& name, const BT::NodeConfiguration& config,
                           const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::ConditionNode>(name, config, shared_resources)
{
}

BT::PortsList HasJsonField::providedPorts()
{
  return { BT::InputPort<std::string>(kPortIDJson, "{json}", "JSON document to check, as JSON text."),
           BT::InputPort<std::string>(kPortIDPath, "JSON Pointer (RFC 6901) of the field to look for.") };
}

BT::KeyValueVector HasJsonField::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "JSON" },
           { moveit_pro::behaviors::kDescriptionMetadataKey, kDescriptionHasJsonField } };
}

BT::NodeStatus HasJsonField::tick()
{
  const auto json_in = json_utils::getJsonText(*this, kPortIDJson);
  const auto path = getInput<std::string>(kPortIDPath);
  if (!json_in || !path)
  {
    shared_resources_->logger->publishFailureMessage(name(), "Failed to get required input: " +
                                                                 (!json_in ? json_in.error() : path.error()));
    return BT::NodeStatus::FAILURE;
  }

  const auto doc = json_utils::parse(json_in.value(), "[json]");
  if (!doc)
  {
    shared_resources_->logger->publishFailureMessage(name(), doc.error());
    return BT::NodeStatus::FAILURE;
  }
  const auto tokens = json_utils::parsePointer(path.value());
  if (!tokens)
  {
    shared_resources_->logger->publishFailureMessage(name(), tokens.error());
    return BT::NodeStatus::FAILURE;
  }
  // A missing field is an ordinary FAILURE for a condition, so it is not logged.
  return json_utils::find(doc.value(), tokens.value()) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
}  // namespace experimental_behaviors
