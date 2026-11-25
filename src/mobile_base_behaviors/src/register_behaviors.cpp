#include <behaviortree_cpp/bt_factory.h>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <mobile_base_behaviors/register_mobile_base_behaviors.hpp>

namespace mobile_base_behaviors
{
class MobileBaseBehaviorsLoader : public moveit_pro::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
                         const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources) override
  {
    registerMobileBaseBehaviors(factory, shared_resources);
  }
};
}  // namespace mobile_base_behaviors

PLUGINLIB_EXPORT_CLASS(mobile_base_behaviors::MobileBaseBehaviorsLoader,
                       moveit_pro::behaviors::SharedResourcesNodeLoaderBase);
