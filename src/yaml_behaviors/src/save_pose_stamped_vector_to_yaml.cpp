// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "yaml_behaviors/save_pose_stamped_vector_to_yaml.hpp"

#include <fstream>

#include <yaml-cpp/yaml.h>
#include <moveit_pro_common/utils/yaml_parsing_tools.hpp>  // YAML::convert<geometry_msgs::msg::PoseStamped>

#include "spdlog/spdlog.h"

namespace yaml_behaviors
{
namespace
{
constexpr auto kPortFilePath = "file_path";
constexpr auto kPortInput = "input";
}  // namespace

SavePoseStampedVectorToYaml::SavePoseStampedVectorToYaml(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList SavePoseStampedVectorToYaml::providedPorts()
{
  return {
    BT::InputPort<std::string>(kPortFilePath, "/tmp/coverage_path.yaml",
                               "Absolute path of the YAML file to write (multi-document: one pose per '---' doc, so "
                               "it loads back with LoadPoseStampedVectorFromYaml)."),
    BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>(
        kPortInput, "{surface_path}", "The vector of PoseStamped (coverage/surface path) to save."),
  };
}

BT::KeyValueVector SavePoseStampedVectorToYaml::metadata()
{
  return { { "subcategory", "User Created Behaviors" },
           { "description",
             "Save a PoseStamped vector (coverage path) to a multi-document YAML file that round-trips with the "
             "built-in LoadPoseStampedVectorFromYaml. Fills the missing vector-save so a captured path can be "
             "replayed deterministically." } };
}

BT::NodeStatus SavePoseStampedVectorToYaml::tick()
{
  const auto file_path = getInput<std::string>(kPortFilePath);
  const auto path = getInput<std::vector<geometry_msgs::msg::PoseStamped>>(kPortInput);
  if (!file_path)
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "SavePoseStampedVectorToYaml: 'file_path' not set.");
    return BT::NodeStatus::FAILURE;
  }
  if (!path)
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "SavePoseStampedVectorToYaml: 'input' not set.");
    return BT::NodeStatus::FAILURE;
  }

  // Emit one YAML document per pose (matches LoadPoseStampedVectorFromYaml -> YAML::LoadAllFromFile,
  // decoding each document as a geometry_msgs::msg::PoseStamped).
  YAML::Emitter out;
  for (const auto& ps : path.value())
  {
    out << YAML::BeginDoc;
    out << YAML::convert<geometry_msgs::msg::PoseStamped>::encode(ps);
  }

  try
  {
    std::ofstream ofs(file_path.value());
    ofs.exceptions(std::ofstream::failbit | std::ofstream::badbit);
    ofs << out.c_str();
    if (!path.value().empty())
    {
      ofs << "\n";
    }
  }
  catch (const std::exception& e)
  {
    getBehaviorContext()->logger->publishFailureMessage(
        name(), std::string("SavePoseStampedVectorToYaml: write failed: ") + e.what());
    return BT::NodeStatus::FAILURE;
  }

  getBehaviorContext()->logger->publishInfoMessage(name(), "SavePoseStampedVectorToYaml: wrote " +
                                                               std::to_string(path.value().size()) + " poses to " +
                                                               file_path.value());
  spdlog::info("SavePoseStampedVectorToYaml: wrote {} poses to {}", path.value().size(), file_path.value());
  return BT::NodeStatus::SUCCESS;
}
}  // namespace yaml_behaviors
