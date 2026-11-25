#include <gtest/gtest.h>

#include <set>
#include <string>

#include <behaviortree_cpp/bt_factory.h>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/node.hpp>

#include <ros_messaging_behaviors/register_ros_messaging_behaviors.hpp>

namespace
{
std::set<std::string> behaviorIds(const BT::BehaviorTreeFactory& factory)
{
  std::set<std::string> ids;
  for (const auto& [id, manifest] : factory.manifests())
  {
    if (factory.builtinNodes().count(id) == 0)
    {
      ids.insert(id);
    }
  }
  return ids;
}
}  // namespace

TEST(BehaviorPlugins, registers_and_instantiates_every_behavior)
{
  pluginlib::ClassLoader<moveit_pro::behaviors::SharedResourcesNodeLoaderBase> loader(
      "moveit_pro_behavior_interface", "moveit_pro::behaviors::SharedResourcesNodeLoaderBase");
  auto node = std::make_shared<rclcpp::Node>("behavior_plugins_test");
  auto context = std::make_shared<moveit_pro::behaviors::BehaviorContext>(node);

  BT::BehaviorTreeFactory expected;
  ASSERT_NO_THROW(ros_messaging_behaviors::registerRosMessagingBehaviors(expected, context));
  BT::BehaviorTreeFactory factory;
  auto plugin = loader.createUniqueInstance("ros_messaging_behaviors::RosMessagingBehaviorsLoader");
  ASSERT_NO_THROW(plugin->registerBehaviors(factory, context));

  const auto ids = behaviorIds(factory);
  EXPECT_FALSE(ids.empty());
  EXPECT_EQ(ids, behaviorIds(expected));
  for (const auto& id : ids)
  {
    EXPECT_NO_THROW((void)factory.instantiateTreeNode("test_" + id, id, BT::NodeConfiguration())) << id;
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
