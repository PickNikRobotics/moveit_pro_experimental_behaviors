#include <behaviortree_cpp/bt_factory.h>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <utility_behaviors/register_utility_behaviors.hpp>

namespace utility_behaviors
{
class UtilityBehaviorsLoader : public moveit_pro::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
                         const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources) override
  {
    registerUtilityBehaviors(factory, shared_resources);
  }
};
}  // namespace utility_behaviors

PLUGINLIB_EXPORT_CLASS(utility_behaviors::UtilityBehaviorsLoader, moveit_pro::behaviors::SharedResourcesNodeLoaderBase);
