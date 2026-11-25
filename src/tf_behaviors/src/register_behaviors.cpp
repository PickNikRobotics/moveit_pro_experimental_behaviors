#include <behaviortree_cpp/bt_factory.h>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <tf_behaviors/register_tf_behaviors.hpp>

namespace tf_behaviors
{
class TfBehaviorsLoader : public moveit_pro::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
                         const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources) override
  {
    registerTfBehaviors(factory, shared_resources);
  }
};
}  // namespace tf_behaviors

PLUGINLIB_EXPORT_CLASS(tf_behaviors::TfBehaviorsLoader, moveit_pro::behaviors::SharedResourcesNodeLoaderBase);
