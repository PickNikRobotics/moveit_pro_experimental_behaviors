// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <experimental_behaviors/json_behaviors/get_json_field.hpp>

#include <experimental_behaviors/json_behaviors/json_utils.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

namespace
{
inline constexpr auto kDescriptionGetJsonField = R"(
                <p>
                    Reads one field of a JSON document. The field is addressed with a JSON Pointer path
                    (RFC 6901), for example <code>/target/pose</code> or <code>/items/0</code>.
                </p>
                <p>
                    A string, boolean or number keeps its type on the blackboard, so typed ports downstream
                    (for example a <code>double</code> or <code>bool</code> port) can read it directly. An
                    object, array or null comes out as JSON text, which other JSON Behaviors can take.
                </p>
                <p>
                    Set <code>message_type</code> to get a ROS message instead, for example
                    <code>geometry_msgs/msg/PoseStamped</code>. Numbers may be JSON numbers or numeric
                    strings, so a pose written by <code>SetJsonField</code> reads back. Fails if the path
                    does not exist or if the field does not match the requested type.
                </p>
            )";

constexpr auto kPortIDJson = "json";
constexpr auto kPortIDPath = "path";
constexpr auto kPortIDMessageType = "message_type";
constexpr auto kPortIDValue = "value";
}  // namespace

namespace experimental_behaviors
{
GetJsonField::GetJsonField(const std::string& name, const BT::NodeConfiguration& config,
                           const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList GetJsonField::providedPorts()
{
  return {
    BT::InputPort<std::string>(kPortIDJson, "{json}", "JSON document to read, as JSON text."),
    BT::InputPort<std::string>(kPortIDPath,
                               "JSON Pointer (RFC 6901) of the field to read, for example /status/battery."),
    BT::InputPort<std::string>(
        kPortIDMessageType, "",
        "Optional ROS message type to convert the field to. Supported: " + json_utils::supportedMessageTypes() + "."),
    BT::OutputPort(kPortIDValue, "{value}", "The field's value."),
  };
}

BT::KeyValueVector GetJsonField::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "JSON" },
           { moveit_pro::behaviors::kDescriptionMetadataKey, kDescriptionGetJsonField } };
}

BT::NodeStatus GetJsonField::tick()
{
  const auto json_in = json_utils::getJsonText(*this, kPortIDJson);
  const auto path = getInput<std::string>(kPortIDPath);
  const auto message_type = getInput<std::string>(kPortIDMessageType);
  if (!json_in || !path || !message_type)
  {
    const std::string error = !json_in ? json_in.error() : (!path ? path.error() : message_type.error());
    shared_resources_->logger->publishFailureMessage(name(), "Failed to get required input: " + error);
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
  const auto field = json_utils::find(doc.value(), tokens.value());
  if (!field)
  {
    shared_resources_->logger->publishFailureMessage(name(), field.error());
    return BT::NodeStatus::FAILURE;
  }

  BT::Any value;
  if (message_type->empty())
  {
    value = json_utils::jsonToAny(**field);
  }
  else
  {
    auto message = json_utils::jsonToMessage(**field, message_type.value(), path.value());
    if (!message)
    {
      shared_resources_->logger->publishFailureMessage(name(), "Cannot read '" + path.value() + "' as " +
                                                                   message_type.value() + ": " + message.error());
      return BT::NodeStatus::FAILURE;
    }
    value = std::move(message.value());
  }

  if (const auto result = setOutput<BT::Any>(kPortIDValue, value); !result)
  {
    shared_resources_->logger->publishFailureMessage(name(), "Failed to set output [value]: " + result.error());
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}
}  // namespace experimental_behaviors
