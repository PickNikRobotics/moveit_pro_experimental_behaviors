// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <behaviortree_cpp/bt_factory.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>

#include "experimental_behaviors/access_interface_value_from_group.hpp"
#include "experimental_behaviors/append_yaml_list_item.hpp"
#include "experimental_behaviors/create_dynamic_interface_group_values.hpp"
#include "experimental_behaviors/create_interface_value.hpp"
#include "experimental_behaviors/get_blackboard_by_key.hpp"
#include "experimental_behaviors/get_dynamic_interface_group_values.hpp"
#include "experimental_behaviors/get_interface_value_from_group.hpp"
#include "experimental_behaviors/get_joint_limits.hpp"
#include "experimental_behaviors/get_pose_stamped_from_topic.hpp"
#include "experimental_behaviors/publish_dynamic_interface_group_values.hpp"
#include "experimental_behaviors/set_blackboard_by_key.hpp"
#include "experimental_behaviors/trajectory_to_path.hpp"
#include "experimental_behaviors/write_yaml_value.hpp"

#include <algorithm>
#include <array>
#include <exception>
#include <set>
#include <string>
#include <tuple>
#include <vector>

#include <pluginlib/class_list_macros.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/node.hpp>

namespace experimental_behaviors
{
namespace
{
constexpr auto kExcludedBehaviorsParameter = "experimental_behaviors.excluded_behaviors";

using RegisterBehavior = void (*)(BT::BehaviorTreeFactory&, const std::string&,
                                  const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>&);

struct BehaviorRegistration
{
  const char* id;
  bool excludable;
  RegisterBehavior register_behavior;
};

// Registration and exclusion eligibility share one inventory; IDs retain their existing ports and implementations.
const std::array<BehaviorRegistration, 13> kBehaviorRegistrations = { {
    { "GetPoseStampedFromTopic", false, &moveit_pro::behaviors::registerBehavior<GetPoseStampedFromTopic> },
    { "CreateDynamicInterfaceGroupValues", true,
      &moveit_pro::behaviors::registerBehavior<CreateDynamicInterfaceGroupValues> },
    { "PublishDynamicInterfaceGroupValues", true,
      &moveit_pro::behaviors::registerBehavior<PublishDynamicInterfaceGroupValues> },
    { "CreateInterfaceValue", true, &moveit_pro::behaviors::registerBehavior<CreateInterfaceValue> },
    { "GetDynamicInterfaceGroupValues", true, &moveit_pro::behaviors::registerBehavior<GetDynamicInterfaceGroupValues> },
    { "GetInterfaceValueFromGroup", true, &moveit_pro::behaviors::registerBehavior<GetInterfaceValueFromGroup> },
    { "AccessInterfaceValue", false, &moveit_pro::behaviors::registerBehavior<AccessInterfaceValueFromGroup> },
    { "GetBlackboardByKey", false, &moveit_pro::behaviors::registerBehavior<GetBlackboardByKey> },
    { "GetJointLimits", false, &moveit_pro::behaviors::registerBehavior<GetJointLimits> },
    { "SetBlackboardByKey", false, &moveit_pro::behaviors::registerBehavior<SetBlackboardByKey> },
    { "TrajectoryToPath", false, &moveit_pro::behaviors::registerBehavior<TrajectoryToPath> },
    { "WriteYamlValue", false, &moveit_pro::behaviors::registerBehavior<WriteYamlValue> },
    { "AppendYamlListItem", false, &moveit_pro::behaviors::registerBehavior<AppendYamlListItem> },
} };

std::set<std::string> validateExcludedBehaviors(const std::vector<std::string>& configured);

std::vector<std::string> readExcludedBehaviors(rclcpp::Node& node)
{
  try
  {
    if (!node.has_parameter(kExcludedBehaviorsParameter))
    {
      const auto& overrides = node.get_node_parameters_interface()->get_parameter_overrides();
      std::vector<std::string> configured;
      if (const auto found = overrides.find(kExcludedBehaviorsParameter); found != overrides.end())
      {
        if (found->second.get_type() != rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
        {
          throw BT::RuntimeError(kExcludedBehaviorsParameter, ": expected a string array");
        }
        configured = found->second.get<std::vector<std::string>>();
      }
      std::ignore = validateExcludedBehaviors(configured);
      rcl_interfaces::msg::ParameterDescriptor descriptor;
      descriptor.read_only = true;
      descriptor.description = "Experimental Behavior IDs supplied by another deployed loader instead.";
      node.declare_parameter<std::vector<std::string>>(kExcludedBehaviorsParameter, std::vector<std::string>{},
                                                       descriptor);
    }

    const auto descriptor = node.describe_parameter(kExcludedBehaviorsParameter);
    const auto parameter = node.get_parameter(kExcludedBehaviorsParameter);
    if (!descriptor.read_only || parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
    {
      throw BT::RuntimeError(kExcludedBehaviorsParameter, ": expected a read-only string array");
    }
    return parameter.as_string_array();
  }
  catch (const std::exception& ex)
  {
    throw BT::RuntimeError(kExcludedBehaviorsParameter, ": invalid startup parameter: ", ex.what());
  }
}

std::set<std::string> validateExcludedBehaviors(const std::vector<std::string>& configured)
{
  std::set<std::string> excluded;
  for (const auto& id : configured)
  {
    if (id.empty())
    {
      throw BT::RuntimeError(kExcludedBehaviorsParameter, ": Behavior IDs must not be empty");
    }
    const auto registration = std::find_if(kBehaviorRegistrations.begin(), kBehaviorRegistrations.end(),
                                           [&id](const auto& entry) { return id == entry.id; });
    if (registration == kBehaviorRegistrations.end() || !registration->excludable)
    {
      throw BT::RuntimeError(kExcludedBehaviorsParameter, ": Behavior ID is not eligible for exclusion: ", id);
    }
    if (!excluded.insert(id).second)
    {
      throw BT::RuntimeError(kExcludedBehaviorsParameter, ": duplicate Behavior ID: ", id);
    }
  }
  return excluded;
}
}  // namespace

class ExperimentalBehaviorsLoader : public moveit_pro::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
                         const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources) override
  {
    if (!shared_resources || !shared_resources->node)
    {
      throw BT::RuntimeError(kExcludedBehaviorsParameter, ": a BehaviorContext with a node is required");
    }
    // Validate the configured exclusions before changing the factory. Exclusions never depend on loader order.
    const auto excluded = validateExcludedBehaviors(readExcludedBehaviors(*shared_resources->node));
    for (const auto& registration : kBehaviorRegistrations)
    {
      if (excluded.count(registration.id) == 0)
      {
        registration.register_behavior(factory, registration.id, shared_resources);
      }
    }
  }
};
}  // namespace experimental_behaviors

PLUGINLIB_EXPORT_CLASS(experimental_behaviors::ExperimentalBehaviorsLoader,
                       moveit_pro::behaviors::SharedResourcesNodeLoaderBase);
