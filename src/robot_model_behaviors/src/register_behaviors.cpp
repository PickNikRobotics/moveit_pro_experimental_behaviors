#include <behaviortree_cpp/bt_factory.h>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <robot_model_behaviors/register_robot_model_behaviors.hpp>

namespace robot_model_behaviors
{
class RobotModelBehaviorsLoader : public moveit_pro::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
                         const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources) override
  {
    registerRobotModelBehaviors(factory, shared_resources);
  }
};
}  // namespace robot_model_behaviors

PLUGINLIB_EXPORT_CLASS(robot_model_behaviors::RobotModelBehaviorsLoader,
                       moveit_pro::behaviors::SharedResourcesNodeLoaderBase);
