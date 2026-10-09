// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <experimental_behaviors/json_behaviors/merge_json.hpp>

#include <experimental_behaviors/json_behaviors/json_utils.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

namespace
{
inline constexpr auto kDescriptionMergeJson = R"(
                <p>
                    Applies a JSON Merge Patch (RFC 7386) to a JSON document, to change several fields in
                    one step.
                </p>
                <p>
                    Objects in <code>patch</code> merge into <code>json</code> key by key, at every level.
                    A key set to <code>null</code> in the patch is removed. Any other value, including an
                    array, replaces the old value. For example, the patch
                    <code>{"status": {"state": "busy"}, "error": null}</code> sets one nested field and
                    removes <code>error</code>. Fails if either input is not valid JSON.
                </p>
            )";

constexpr auto kPortIDJson = "json";
constexpr auto kPortIDPatch = "patch";
}  // namespace

namespace experimental_behaviors
{
MergeJson::MergeJson(const std::string& name, const BT::NodeConfiguration& config,
                     const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList MergeJson::providedPorts()
{
  return { BT::BidirectionalPort<std::string>(kPortIDJson, "{json}", "JSON document to edit, as JSON text."),
           BT::InputPort<std::string>(kPortIDPatch, "JSON Merge Patch (RFC 7386) to apply, as JSON text.") };
}

BT::KeyValueVector MergeJson::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "JSON" },
           { moveit_pro::behaviors::kDescriptionMetadataKey, kDescriptionMergeJson } };
}

BT::NodeStatus MergeJson::tick()
{
  const auto json_in = getInput<std::string>(kPortIDJson);
  const auto patch_in = json_utils::getJsonText(*this, kPortIDPatch);
  if (!json_in || !patch_in)
  {
    shared_resources_->logger->publishFailureMessage(name(), "Failed to get required input: " +
                                                                 (!json_in ? json_in.error() : patch_in.error()));
    return BT::NodeStatus::FAILURE;
  }

  auto doc = json_utils::parse(json_in.value(), "[json]");
  if (!doc)
  {
    shared_resources_->logger->publishFailureMessage(name(), doc.error());
    return BT::NodeStatus::FAILURE;
  }
  const auto patch = json_utils::parse(patch_in.value(), "[patch]");
  if (!patch)
  {
    shared_resources_->logger->publishFailureMessage(name(), patch.error());
    return BT::NodeStatus::FAILURE;
  }
  doc->merge_patch(patch.value());
  setOutput(kPortIDJson, doc->dump());
  return BT::NodeStatus::SUCCESS;
}
}  // namespace experimental_behaviors
