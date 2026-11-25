#include <behaviortree_cpp/bt_factory.h>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <blackboard_behaviors/register_blackboard_behaviors.hpp>

namespace blackboard_behaviors
{
class BlackboardBehaviorsLoader : public moveit_pro::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
                         const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources) override
  {
    registerBlackboardBehaviors(factory, shared_resources);
  }
};
}  // namespace blackboard_behaviors

PLUGINLIB_EXPORT_CLASS(blackboard_behaviors::BlackboardBehaviorsLoader,
                       moveit_pro::behaviors::SharedResourcesNodeLoaderBase);
