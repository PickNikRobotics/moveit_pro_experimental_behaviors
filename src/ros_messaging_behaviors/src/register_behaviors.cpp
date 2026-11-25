#include <behaviortree_cpp/bt_factory.h>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <ros_messaging_behaviors/register_ros_messaging_behaviors.hpp>

namespace ros_messaging_behaviors
{
class RosMessagingBehaviorsLoader : public moveit_pro::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
                         const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources) override
  {
    registerRosMessagingBehaviors(factory, shared_resources);
  }
};
}  // namespace ros_messaging_behaviors

PLUGINLIB_EXPORT_CLASS(ros_messaging_behaviors::RosMessagingBehaviorsLoader,
                       moveit_pro::behaviors::SharedResourcesNodeLoaderBase);
