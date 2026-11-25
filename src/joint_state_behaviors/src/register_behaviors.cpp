#include <behaviortree_cpp/bt_factory.h>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <joint_state_behaviors/register_joint_state_behaviors.hpp>

namespace joint_state_behaviors
{
class JointStateBehaviorsLoader : public moveit_pro::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
                         const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources) override
  {
    registerJointStateBehaviors(factory, shared_resources);
  }
};
}  // namespace joint_state_behaviors

PLUGINLIB_EXPORT_CLASS(joint_state_behaviors::JointStateBehaviorsLoader,
                       moveit_pro::behaviors::SharedResourcesNodeLoaderBase);
