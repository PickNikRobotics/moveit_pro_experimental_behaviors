#include <behaviortree_cpp/bt_factory.h>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <point_cloud_behaviors/register_point_cloud_behaviors.hpp>

namespace point_cloud_behaviors
{
class PointCloudBehaviorsLoader : public moveit_pro::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
                         const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources) override
  {
    registerPointCloudBehaviors(factory, shared_resources);
  }
};
}  // namespace point_cloud_behaviors

PLUGINLIB_EXPORT_CLASS(point_cloud_behaviors::PointCloudBehaviorsLoader,
                       moveit_pro::behaviors::SharedResourcesNodeLoaderBase);
