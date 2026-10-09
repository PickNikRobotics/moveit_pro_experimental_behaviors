// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <experimental_behaviors/json_behaviors/remove_json_field.hpp>

#include <experimental_behaviors/json_behaviors/json_utils.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

namespace
{
inline constexpr auto kDescriptionRemoveJsonField = R"(
                <p>
                    Removes one field of a JSON document, addressed with a JSON Pointer path (RFC 6901).
                    Removing an array element shifts the later elements down by one.
                </p>
                <p>
                    Succeeds without changing the document if the field does not exist. Fails on invalid
                    JSON, an invalid path, or the empty path (the whole document cannot be removed).
                </p>
            )";

constexpr auto kPortIDJson = "json";
constexpr auto kPortIDPath = "path";
}  // namespace

namespace experimental_behaviors
{
RemoveJsonField::RemoveJsonField(const std::string& name, const BT::NodeConfiguration& config,
                                 const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList RemoveJsonField::providedPorts()
{
  return { BT::BidirectionalPort<std::string>(kPortIDJson, "{json}", "JSON document to edit, as JSON text."),
           BT::InputPort<std::string>(kPortIDPath, "JSON Pointer (RFC 6901) of the field to remove.") };
}

BT::KeyValueVector RemoveJsonField::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "JSON" },
           { moveit_pro::behaviors::kDescriptionMetadataKey, kDescriptionRemoveJsonField } };
}

BT::NodeStatus RemoveJsonField::tick()
{
  const auto json_in = getInput<std::string>(kPortIDJson);
  const auto path = getInput<std::string>(kPortIDPath);
  if (!json_in || !path)
  {
    shared_resources_->logger->publishFailureMessage(name(), "Failed to get required input: " +
                                                                 (!json_in ? json_in.error() : path.error()));
    return BT::NodeStatus::FAILURE;
  }

  auto doc = json_utils::parse(json_in.value(), "[json]");
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
  const auto removed = json_utils::remove(doc.value(), tokens.value());
  if (!removed)
  {
    shared_resources_->logger->publishFailureMessage(name(), removed.error());
    return BT::NodeStatus::FAILURE;
  }
  if (removed.value())
  {
    setOutput(kPortIDJson, doc->dump());
  }
  return BT::NodeStatus::SUCCESS;
}
}  // namespace experimental_behaviors
