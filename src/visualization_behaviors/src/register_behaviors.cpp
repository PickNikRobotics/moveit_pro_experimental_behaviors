#include <behaviortree_cpp/bt_factory.h>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <visualization_behaviors/register_visualization_behaviors.hpp>

namespace visualization_behaviors
{
class VisualizationBehaviorsLoader : public moveit_pro::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
                         const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources) override
  {
    registerVisualizationBehaviors(factory, shared_resources);
  }
};
}  // namespace visualization_behaviors

PLUGINLIB_EXPORT_CLASS(visualization_behaviors::VisualizationBehaviorsLoader,
                       moveit_pro::behaviors::SharedResourcesNodeLoaderBase);
