#include <behaviortree_cpp/bt_factory.h>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <control_interface_behaviors/register_control_interface_behaviors.hpp>

namespace control_interface_behaviors
{
class ControlInterfaceBehaviorsLoader : public moveit_pro::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
                         const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources) override
  {
    registerControlInterfaceBehaviors(factory, shared_resources);
  }
};
}  // namespace control_interface_behaviors

PLUGINLIB_EXPORT_CLASS(control_interface_behaviors::ControlInterfaceBehaviorsLoader,
                       moveit_pro::behaviors::SharedResourcesNodeLoaderBase);
