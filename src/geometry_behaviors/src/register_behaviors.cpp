#include <behaviortree_cpp/bt_factory.h>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <geometry_behaviors/register_geometry_behaviors.hpp>

namespace geometry_behaviors
{
class GeometryBehaviorsLoader : public moveit_pro::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
                         const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources) override
  {
    registerGeometryBehaviors(factory, shared_resources);
  }
};
}  // namespace geometry_behaviors

PLUGINLIB_EXPORT_CLASS(geometry_behaviors::GeometryBehaviorsLoader,
                       moveit_pro::behaviors::SharedResourcesNodeLoaderBase);
