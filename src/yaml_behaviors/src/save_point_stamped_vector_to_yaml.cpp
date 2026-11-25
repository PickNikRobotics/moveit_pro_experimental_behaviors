// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "yaml_behaviors/save_point_stamped_vector_to_yaml.hpp"

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>
#include <moveit_pro_common/utils/filesystem_utils.hpp>
#include <moveit_pro_common/utils/yaml_parsing_tools.hpp>

namespace
{
constexpr auto kPortIDYamlFilename = "yaml_filename";
constexpr auto kPortIDOverwrite = "overwrite";
constexpr auto kPortIDMessage = "message";

constexpr auto kConfigSourceDirectory = "config_source_directory";

inline constexpr auto kDescription = R"(
    <p>Saves a vector of <code>geometry_msgs::msg::PointStamped</code> messages to a multi-document
    YAML file in the objectives directory. Each message is written as a separate YAML document,
    compatible with <code>LoadPointStampedVectorFromYaml</code>.</p>
)";

tl::expected<void, std::string> writeToYaml(const std::string& filepath, bool overwrite,
                                            const std::vector<geometry_msgs::msg::PointStamped>& message)
{
  if (!overwrite && std::filesystem::exists(filepath))
  {
    return tl::make_unexpected(std::string("File already exists and overwrite is false."));
  }

  try
  {
    std::ofstream output_stream(filepath);
    output_stream.exceptions(std::ofstream::failbit | std::ofstream::badbit);

    for (const auto& point_stamped : message)
    {
      YAML::Emitter emitter;
      emitter << YAML::convert<geometry_msgs::msg::PointStamped>::encode(point_stamped);
      output_stream << "---\n" << emitter.c_str() << "\n";
    }
  }
  catch (const std::exception& e)
  {
    return tl::make_unexpected(std::string(e.what()));
  }
  return {};
}

}  // namespace

namespace yaml_behaviors
{

SavePointStampedVectorToYaml::SavePointStampedVectorToYaml(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : SharedResourcesNode(name, config, shared_resources)
{
}

BT::PortsList SavePointStampedVectorToYaml::providedPorts()
{
  return {
    BT::InputPort<std::string>(kPortIDYamlFilename, "point_stamped_vector", "The name of the YAML file to save."),
    BT::InputPort<bool>(kPortIDOverwrite, true, "Overwrite the file if it already exists."),
    BT::InputPort<std::vector<geometry_msgs::msg::PointStamped>>(kPortIDMessage, "{point_stamped_vector}",
                                                                 "A vector of geometry_msgs::msg::PointStamped "
                                                                 "messages to write to the file."),
  };
}

BT::KeyValueVector SavePointStampedVectorToYaml::metadata()
{
  return { { moveit_pro::behaviors::kDescriptionMetadataKey, kDescription },
           { moveit_pro::behaviors::kSubcategoryMetadataKey, "YAML Handling" } };
}

BT::NodeStatus SavePointStampedVectorToYaml::tick()
{
  const auto ports =
      moveit_pro::behaviors::getRequiredInputs(getInput<std::string>(kPortIDYamlFilename),
                                               getInput<std::vector<geometry_msgs::msg::PointStamped>>(kPortIDMessage));

  if (!ports.has_value())
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "Failed to get required value from input data port: " +
                                                                    ports.error());
    return BT::NodeStatus::FAILURE;
  }

  const auto& [yaml_filename, message] = ports.value();

  const bool overwrite = getInput<bool>(kPortIDOverwrite).value_or(true);

  // Resolve the file path relative to the objectives source directory.
  std::string objectives_source_directory;
  try
  {
    objectives_source_directory = getBehaviorContext()->node->get_parameter(kConfigSourceDirectory).as_string();
  }
  catch (const rclcpp::exceptions::ParameterNotDeclaredException& ex)
  {
    getBehaviorContext()->logger->publishFailureMessage(
        name(), std::string("Parameter 'config_source_directory' not declared: ") + ex.what());
    return BT::NodeStatus::FAILURE;
  }

  auto path_expected = moveit_pro::common::filesystem_utils::getFilePath(yaml_filename, objectives_source_directory);
  if (!path_expected.has_value())
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "Failed to get filepath: " + path_expected.error());
    return BT::NodeStatus::FAILURE;
  }

  std::filesystem::path filepath(path_expected.value());
  if (filepath.extension().empty())
  {
    filepath.replace_extension(".yaml");
  }

  if (!std::filesystem::exists(filepath.parent_path()))
  {
    try
    {
      std::filesystem::create_directories(filepath.parent_path());
    }
    catch (const std::filesystem::filesystem_error& ex)
    {
      getBehaviorContext()->logger->publishFailureMessage(
          name(), "Failed to create parent directory '" + filepath.parent_path().string() + "': " + ex.what());
      return BT::NodeStatus::FAILURE;
    }
  }

  const auto save_result = writeToYaml(filepath.string(), overwrite, message);
  if (!save_result.has_value())
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "Failed saving to '" + filepath.string() +
                                                                    "': " + save_result.error());
    return BT::NodeStatus::FAILURE;
  }

  getBehaviorContext()->logger->publishInfoMessage(
      name(), "Saved " + std::to_string(message.size()) + " PointStamped messages to '" + filepath.string() + "'.");

  return BT::NodeStatus::SUCCESS;
}

}  // namespace yaml_behaviors
