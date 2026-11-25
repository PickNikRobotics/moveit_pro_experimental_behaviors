#include <behaviortree_cpp/bt_factory.h>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <hardware_behaviors/register_hardware_behaviors.hpp>

namespace hardware_behaviors
{
class HardwareBehaviorsLoader : public moveit_pro::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
                         const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources) override
  {
    registerHardwareBehaviors(factory, shared_resources);
  }
};
}  // namespace hardware_behaviors

PLUGINLIB_EXPORT_CLASS(hardware_behaviors::HardwareBehaviorsLoader,
                       moveit_pro::behaviors::SharedResourcesNodeLoaderBase);
