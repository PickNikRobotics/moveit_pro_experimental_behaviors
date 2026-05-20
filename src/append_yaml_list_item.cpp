// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <experimental_behaviors/append_yaml_list_item.hpp>

#include <yaml-cpp/yaml.h>
#include <experimental_behaviors/path_expansion.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace
{
inline constexpr auto kDescriptionAppendYamlListItem = R"(
                <p>
                    Appends a value to a sequence at a nested key path in a YAML file. Companion to
                    <code>WriteYamlValue</code> (set) and <code>ReadYamlList</code> (read).
                    Takes the file location as a single <code>file_path</code> port that accepts
                    an absolute path, an absolute path with <code>${VAR}</code> / leading
                    <code>~</code> expansion, or a ROS <code>package://&lt;pkg&gt;/&lt;rest&gt;</code> URL.
                </p>
                <p>
                    The keyed location must either be absent (a new sequence is created) or already
                    be a sequence; appending into a scalar or map fails the tick. The
                    <code>value</code> port is parsed as YAML before being appended, so scalars
                    and inline maps (<code>"{ key: val, ... }"</code>) both work through the same
                    string port.
                </p>
                <p>
                    Round-trips through yaml-cpp on every tick: load existing &rarr; modify &rarr; dump.
                    Fails the tick if the file does not exist.
                </p>
            )";

constexpr auto kPortIDFilePath = "file_path";
constexpr auto kPortIDKey1 = "key1";
constexpr auto kPortIDKey2 = "key2";
constexpr auto kPortIDKey3 = "key3";
constexpr auto kPortIDKey4 = "key4";
constexpr auto kPortIDKey5 = "key5";
constexpr auto kPortIDValue = "value";
}  // namespace

namespace experimental_behaviors
{
AppendYamlListItem::AppendYamlListItem(const std::string& name, const BT::NodeConfiguration& config,
                                       const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList AppendYamlListItem::providedPorts()
{
  return { BT::InputPort<std::string>(kPortIDFilePath,
                                      "Path to the YAML file. Accepts an absolute path "
                                      "(/home/user/foo.yaml), an absolute path with shell-style ${VAR} / "
                                      "leading ~ expansion (${HOME}/foo.yaml), or a ROS package URL "
                                      "(package://my_pkg/config/foo.yaml resolves via "
                                      "ament_index_cpp::get_package_share_directory)."),
           BT::InputPort<std::string>(kPortIDKey1, "First key to traverse YAML structure (mandatory)"),
           BT::InputPort<std::string>(kPortIDKey2, "", "Second key (optional)"),
           BT::InputPort<std::string>(kPortIDKey3, "", "Third key (optional)"),
           BT::InputPort<std::string>(kPortIDKey4, "", "Fourth key (optional)"),
           BT::InputPort<std::string>(kPortIDKey5, "", "Fifth key (optional)"),
           BT::InputPort<std::string>(kPortIDValue, "Value to append. Parsed as YAML, so scalars and inline maps "
                                                    "(`{ key: val, ... }`) both work through the same string port.") };
}

BT::KeyValueVector AppendYamlListItem::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "YAML Operations" },
           { moveit_pro::behaviors::kDescriptionMetadataKey, kDescriptionAppendYamlListItem } };
}

BT::NodeStatus AppendYamlListItem::tick()
{
  const auto& logger = shared_resources_->node->get_logger();

  const auto file_path_in = getInput<std::string>(kPortIDFilePath);
  const auto key1_in = getInput<std::string>(kPortIDKey1);
  const auto value_in = getInput<std::string>(kPortIDValue);

  if (!file_path_in || file_path_in->empty())
  {
    RCLCPP_ERROR(logger, "AppendYamlListItem: 'file_path' is required.");
    return BT::NodeStatus::FAILURE;
  }
  if (!key1_in || key1_in->empty())
  {
    RCLCPP_ERROR(logger, "AppendYamlListItem: 'key1' is required.");
    return BT::NodeStatus::FAILURE;
  }
  if (!value_in)
  {
    RCLCPP_ERROR(logger, "AppendYamlListItem: 'value' is required (empty string allowed).");
    return BT::NodeStatus::FAILURE;
  }

  const std::string file_path = expandPath(file_path_in.value());

  std::vector<std::string> keys;
  keys.push_back(key1_in.value());
  for (const char* port_id : { kPortIDKey2, kPortIDKey3, kPortIDKey4, kPortIDKey5 })
  {
    auto v = getInput<std::string>(port_id);
    if (v && !v->empty())
    {
      keys.push_back(v.value());
    }
  }

  if (!std::filesystem::exists(file_path))
  {
    RCLCPP_ERROR(logger, "AppendYamlListItem: file does not exist: %s", file_path.c_str());
    return BT::NodeStatus::FAILURE;
  }

  YAML::Node root;
  try
  {
    root = YAML::LoadFile(file_path);
  }
  catch (const YAML::Exception& e)
  {
    RCLCPP_ERROR(logger, "AppendYamlListItem: failed to parse '%s': %s", file_path.c_str(), e.what());
    return BT::NodeStatus::FAILURE;
  }

  YAML::Node parsed_value;
  try
  {
    parsed_value = YAML::Load(value_in.value());
  }
  catch (const YAML::Exception& e)
  {
    RCLCPP_ERROR(logger, "AppendYamlListItem: failed to parse 'value' as YAML: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }

  // Traverse to the parent of the leaf key, creating intermediate maps as needed.
  YAML::Node node = root;
  for (size_t i = 0; i + 1 < keys.size(); ++i)
  {
    const auto& k = keys[i];
    if (!node[k] || node[k].IsNull())
    {
      node[k] = YAML::Node(YAML::NodeType::Map);
    }
    node = node[k];
  }

  // Ensure the leaf is a sequence (creating it if absent).
  const auto& leaf = keys.back();
  if (!node[leaf] || node[leaf].IsNull())
  {
    node[leaf] = YAML::Node(YAML::NodeType::Sequence);
  }
  if (!node[leaf].IsSequence())
  {
    RCLCPP_ERROR(logger, "AppendYamlListItem: target at the keyed location exists and is not a sequence.");
    return BT::NodeStatus::FAILURE;
  }
  node[leaf].push_back(parsed_value);

  try
  {
    std::ofstream out(file_path);
    if (!out)
    {
      RCLCPP_ERROR(logger, "AppendYamlListItem: failed to open '%s' for writing.", file_path.c_str());
      return BT::NodeStatus::FAILURE;
    }
    out << root;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(logger, "AppendYamlListItem: failed to write '%s': %s", file_path.c_str(), e.what());
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}
}  // namespace experimental_behaviors
