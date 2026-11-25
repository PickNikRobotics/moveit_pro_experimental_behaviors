// Functional coverage for GenerateShapeUrdf — preserves the per-shape and
// failure-mode tests that lived in the pre-consolidation
// generate_shape_urdf/test/test_behavior_plugins.cpp.
//
// The plugin-discovery smoke test (`LoadAndInstantiate`) is covered by the
// package-wide test_behavior_plugins.cpp; this file focuses on input/output
// semantics.

#include <gtest/gtest.h>

#include <behaviortree_cpp/bt_factory.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <rclcpp/node.hpp>

#include "planning_scene_behaviors/register_planning_scene_behaviors.hpp"

namespace
{
class GenerateShapeUrdfTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    planning_scene_behaviors::registerPlanningSceneBehaviors(
        factory_,
        std::make_shared<moveit_pro::behaviors::BehaviorContext>(std::make_shared<rclcpp::Node>("BehaviorTests")));
  }

  // Build a NodeConfiguration that wires the named ports to a fresh blackboard
  // pre-populated with the supplied `shape_type`, `dimensions`, and optional
  // `object_name` (defaults to the behavior's port default 'generated_object' when omitted).
  BT::NodeConfiguration makeConfig(const std::string& shape_type, const std::vector<double>& dimensions,
                                   const std::string& object_name = "")
  {
    BT::NodeConfiguration config;
    config.blackboard = BT::Blackboard::create();
    config.blackboard->set("shape_type", shape_type);
    config.blackboard->set("dimensions", dimensions);
    config.input_ports["shape_type"] = "{shape_type}";
    config.input_ports["dimensions"] = "{dimensions}";
    config.output_ports["urdf_string"] = "{urdf_string}";
    if (!object_name.empty())
    {
      config.blackboard->set("object_name", object_name);
      config.input_ports["object_name"] = "{object_name}";
    }
    return config;
  }

  BT::BehaviorTreeFactory factory_;
};
}  // namespace

TEST_F(GenerateShapeUrdfTest, GenerateBoxUrdf)
{
  auto config = makeConfig("box", { 0.1, 0.2, 0.3 }, "generated_box");
  auto behavior = factory_.instantiateTreeNode("test", "GenerateShapeUrdf", config);
  ASSERT_EQ(behavior->executeTick(), BT::NodeStatus::SUCCESS);

  const auto urdf_string = config.blackboard->get<std::string>("urdf_string");
  EXPECT_NE(urdf_string.find("<robot name=\"generated_box\">"), std::string::npos);
  EXPECT_NE(urdf_string.find("<box size=\"0.1 0.2 0.3\"/>"), std::string::npos);
}

TEST_F(GenerateShapeUrdfTest, GenerateSphereUrdf)
{
  auto config = makeConfig("sphere", { 0.05 }, "generated_sphere");
  auto behavior = factory_.instantiateTreeNode("test", "GenerateShapeUrdf", config);
  ASSERT_EQ(behavior->executeTick(), BT::NodeStatus::SUCCESS);

  const auto urdf_string = config.blackboard->get<std::string>("urdf_string");
  EXPECT_NE(urdf_string.find("<robot name=\"generated_sphere\">"), std::string::npos);
  EXPECT_NE(urdf_string.find("<sphere radius=\"0.05\"/>"), std::string::npos);
}

TEST_F(GenerateShapeUrdfTest, GenerateCylinderUrdf)
{
  auto config = makeConfig("cylinder", { 0.05, 0.2 }, "generated_cylinder");
  auto behavior = factory_.instantiateTreeNode("test", "GenerateShapeUrdf", config);
  ASSERT_EQ(behavior->executeTick(), BT::NodeStatus::SUCCESS);

  const auto urdf_string = config.blackboard->get<std::string>("urdf_string");
  EXPECT_NE(urdf_string.find("<robot name=\"generated_cylinder\">"), std::string::npos);
  EXPECT_NE(urdf_string.find("<cylinder radius=\"0.05\" length=\"0.2\"/>"), std::string::npos);
}

TEST_F(GenerateShapeUrdfTest, InvalidShapeTypeFails)
{
  auto config = makeConfig("triangle", { 0.1 });
  auto behavior = factory_.instantiateTreeNode("test", "GenerateShapeUrdf", config);
  EXPECT_EQ(behavior->executeTick(), BT::NodeStatus::FAILURE);
}

TEST_F(GenerateShapeUrdfTest, WrongDimensionCountFails)
{
  auto config = makeConfig("box", { 0.1, 0.2 });  // box needs 3
  auto behavior = factory_.instantiateTreeNode("test", "GenerateShapeUrdf", config);
  EXPECT_EQ(behavior->executeTick(), BT::NodeStatus::FAILURE);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
