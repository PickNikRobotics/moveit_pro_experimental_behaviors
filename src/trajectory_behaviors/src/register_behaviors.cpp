#include <behaviortree_cpp/bt_factory.h>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <trajectory_behaviors/register_trajectory_behaviors.hpp>

namespace trajectory_behaviors
{
class TrajectoryBehaviorsLoader : public moveit_pro::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
                         const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources) override
  {
    registerTrajectoryBehaviors(factory, shared_resources);
  }
};
}  // namespace trajectory_behaviors

PLUGINLIB_EXPORT_CLASS(trajectory_behaviors::TrajectoryBehaviorsLoader,
                       moveit_pro::behaviors::SharedResourcesNodeLoaderBase);
