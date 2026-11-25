#include <behaviortree_cpp/bt_factory.h>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <mtc_behaviors/register_mtc_behaviors.hpp>

namespace mtc_behaviors
{
class MtcBehaviorsLoader : public moveit_pro::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
                         const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources) override
  {
    registerMtcBehaviors(factory, shared_resources);
  }
};
}  // namespace mtc_behaviors

PLUGINLIB_EXPORT_CLASS(mtc_behaviors::MtcBehaviorsLoader, moveit_pro::behaviors::SharedResourcesNodeLoaderBase);
