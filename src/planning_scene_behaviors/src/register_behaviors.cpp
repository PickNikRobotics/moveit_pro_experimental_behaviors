#include <behaviortree_cpp/bt_factory.h>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <planning_scene_behaviors/register_planning_scene_behaviors.hpp>

namespace planning_scene_behaviors
{
class PlanningSceneBehaviorsLoader : public moveit_pro::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
                         const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources) override
  {
    registerPlanningSceneBehaviors(factory, shared_resources);
  }
};
}  // namespace planning_scene_behaviors

PLUGINLIB_EXPORT_CLASS(planning_scene_behaviors::PlanningSceneBehaviorsLoader,
                       moveit_pro::behaviors::SharedResourcesNodeLoaderBase);
