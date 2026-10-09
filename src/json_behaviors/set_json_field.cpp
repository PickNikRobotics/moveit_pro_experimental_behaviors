// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <experimental_behaviors/json_behaviors/set_json_field.hpp>

#include <experimental_behaviors/json_behaviors/json_utils.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

namespace
{
inline constexpr auto kDescriptionSetJsonField = R"(
                <p>
                    Adds or replaces one field of a JSON document. The field is addressed with a JSON
                    Pointer path (RFC 6901): <code>/status/battery</code> is a nested key,
                    <code>/items/0</code> is the first array element, <code>/items/-</code> appends to
                    an array, <code>~1</code> stands for <code>/</code> and <code>~0</code> for
                    <code>~</code> inside a key. The empty path replaces the whole document.
                </p>
                <p>
                    <code>value</code> takes any type. Strings, booleans and numbers on the blackboard
                    become the matching JSON type. ROS messages, such as a PoseStamped, are converted with
                    the same JSON converters the blackboard view in the UI uses, so the result matches what
                    the UI shows. A value typed directly into the port is a JSON string, unless
                    <code>parse_value_as_json</code> is true; then it is parsed as JSON, so
                    <code>42</code>, <code>true</code> or <code>{"a": 1}</code> keep their type.
                </p>
                <p>
                    With <code>create_missing</code> true (the default), missing parent objects are created.
                    Fails on invalid JSON, an invalid path, a parent that is a string, number or boolean,
                    or a value whose C++ type has no JSON converter.
                </p>
            )";

constexpr auto kPortIDJson = "json";
constexpr auto kPortIDPath = "path";
constexpr auto kPortIDValue = "value";
constexpr auto kPortIDCreateMissing = "create_missing";
constexpr auto kPortIDParseValueAsJson = "parse_value_as_json";
}  // namespace

namespace experimental_behaviors
{
SetJsonField::SetJsonField(const std::string& name, const BT::NodeConfiguration& config,
                           const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList SetJsonField::providedPorts()
{
  return {
    BT::BidirectionalPort<std::string>(kPortIDJson, "{json}", "JSON document to edit, as JSON text."),
    BT::InputPort<std::string>(kPortIDPath,
                               "JSON Pointer (RFC 6901) of the field to set, for example /status/battery."),
    BT::InputPort(kPortIDValue, "Value to store. A blackboard entry of any type, or a literal typed into the port."),
    BT::InputPort<bool>(kPortIDCreateMissing, true, "If true, create missing parent objects along the path."),
    BT::InputPort<bool>(kPortIDParseValueAsJson, false,
                        "If true, parse a string value as JSON instead of storing it as a JSON string."),
  };
}

BT::KeyValueVector SetJsonField::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "JSON" },
           { moveit_pro::behaviors::kDescriptionMetadataKey, kDescriptionSetJsonField } };
}

BT::NodeStatus SetJsonField::tick()
{
  const auto json_in = getInput<std::string>(kPortIDJson);
  const auto path = getInput<std::string>(kPortIDPath);
  const auto create_missing = getInput<bool>(kPortIDCreateMissing);
  const auto parse_value_as_json = getInput<bool>(kPortIDParseValueAsJson);
  if (!json_in || !path || !create_missing || !parse_value_as_json)
  {
    const std::string error = !json_in ? json_in.error() :
                              !path    ? path.error() :
                                         (!create_missing ? create_missing.error() : parse_value_as_json.error());
    shared_resources_->logger->publishFailureMessage(name(), "Failed to get required input: " + error);
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

  // Read the raw port: getInput would turn a blackboard value into a string and lose its type.
  BT::Any value;
  const auto port_it = config().input_ports.find(kPortIDValue);
  if (port_it == config().input_ports.end() || port_it->second.empty())
  {
    shared_resources_->logger->publishFailureMessage(name(), "Missing input port [value].");
    return BT::NodeStatus::FAILURE;
  }
  BT::StringView key;
  if (json_utils::isJsonLiteralPort(*this, kPortIDValue))
  {
    value = BT::Any(port_it->second);
  }
  else if (BT::TreeNode::isBlackboardPointer(port_it->second, &key))
  {
    const auto entry = config().blackboard->getEntry(std::string(key));
    if (!entry)
    {
      shared_resources_->logger->publishFailureMessage(name(), "[value] points to blackboard entry '" +
                                                                   std::string(key) + "', which does not exist.");
      return BT::NodeStatus::FAILURE;
    }
    std::scoped_lock lock(entry->entry_mutex);
    value = entry->value;
  }
  else
  {
    value = BT::Any(port_it->second);
  }

  tl::expected<json_utils::Json, std::string> json_value;
  if (parse_value_as_json.value() && value.isString())
  {
    json_value = json_utils::parse(value.cast<std::string>(), "[value]");
  }
  else
  {
    json_value = json_utils::anyToJson(value);
  }
  if (!json_value)
  {
    shared_resources_->logger->publishFailureMessage(name(), "Cannot store [value]: " + json_value.error());
    return BT::NodeStatus::FAILURE;
  }

  if (const auto result =
          json_utils::set(doc.value(), tokens.value(), std::move(json_value.value()), create_missing.value());
      !result)
  {
    shared_resources_->logger->publishFailureMessage(name(), result.error());
    return BT::NodeStatus::FAILURE;
  }
  setOutput(kPortIDJson, doc->dump());
  return BT::NodeStatus::SUCCESS;
}
}  // namespace experimental_behaviors
