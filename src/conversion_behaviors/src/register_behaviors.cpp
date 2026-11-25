#include <behaviortree_cpp/bt_factory.h>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <conversion_behaviors/register_conversion_behaviors.hpp>

namespace conversion_behaviors
{
class ConversionBehaviorsLoader : public moveit_pro::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
                         const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources) override
  {
    registerConversionBehaviors(factory, shared_resources);
  }
};
}  // namespace conversion_behaviors

PLUGINLIB_EXPORT_CLASS(conversion_behaviors::ConversionBehaviorsLoader,
                       moveit_pro::behaviors::SharedResourcesNodeLoaderBase);
