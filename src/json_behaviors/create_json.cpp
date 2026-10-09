// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <experimental_behaviors/json_behaviors/create_json.hpp>

#include <experimental_behaviors/json_behaviors/json_utils.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

namespace
{
inline constexpr auto kDescriptionCreateJson = R"(
                <p>
                    Creates a JSON document on the blackboard. The document is stored as a string, so
                    any port that takes a string can pass it on.
                </p>
                <p>
                    With no <code>initial</code> value the document is an empty object <code>{}</code>.
                    Otherwise <code>initial</code> is validated and copied, so a template such as
                    <code>{"robot": "arm_1", "status": {}}</code> can be filled in later with
                    <code>SetJsonField</code>. Fails if <code>initial</code> is not valid JSON; the
                    message gives the line and column of the error.
                </p>
            )";

constexpr auto kPortIDInitial = "initial";
constexpr auto kPortIDJson = "json";
}  // namespace

namespace experimental_behaviors
{
CreateJson::CreateJson(const std::string& name, const BT::NodeConfiguration& config,
                       const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList CreateJson::providedPorts()
{
  return { BT::InputPort<std::string>(kPortIDInitial, "{}", "JSON text to start from. Defaults to an empty object."),
           BT::OutputPort<std::string>(kPortIDJson, "{json}", "The new JSON document, as compact JSON text.") };
}

BT::KeyValueVector CreateJson::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "JSON" },
           { moveit_pro::behaviors::kDescriptionMetadataKey, kDescriptionCreateJson } };
}

BT::NodeStatus CreateJson::tick()
{
  const auto initial = json_utils::getJsonText(*this, kPortIDInitial);
  if (!initial)
  {
    shared_resources_->logger->publishFailureMessage(name(), initial.error());
    return BT::NodeStatus::FAILURE;
  }
  const auto doc = json_utils::parse(initial.value(), "[initial]");
  if (!doc)
  {
    shared_resources_->logger->publishFailureMessage(name(), doc.error());
    return BT::NodeStatus::FAILURE;
  }
  setOutput(kPortIDJson, doc->dump());
  return BT::NodeStatus::SUCCESS;
}
}  // namespace experimental_behaviors
