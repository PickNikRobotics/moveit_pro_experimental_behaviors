#include <behaviortree_cpp/bt_factory.h>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <yaml_behaviors/register_yaml_behaviors.hpp>

namespace yaml_behaviors
{
class YamlBehaviorsLoader : public moveit_pro::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
                         const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources) override
  {
    registerYamlBehaviors(factory, shared_resources);
  }
};
}  // namespace yaml_behaviors

PLUGINLIB_EXPORT_CLASS(yaml_behaviors::YamlBehaviorsLoader, moveit_pro::behaviors::SharedResourcesNodeLoaderBase);
