#include <behaviortree_cpp/bt_factory.h>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <grasping_behaviors/register_grasping_behaviors.hpp>

namespace grasping_behaviors
{
class GraspingBehaviorsLoader : public moveit_pro::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
                         const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources) override
  {
    registerGraspingBehaviors(factory, shared_resources);
  }
};
}  // namespace grasping_behaviors

PLUGINLIB_EXPORT_CLASS(grasping_behaviors::GraspingBehaviorsLoader,
                       moveit_pro::behaviors::SharedResourcesNodeLoaderBase);
