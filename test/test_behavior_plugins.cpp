#include <gtest/gtest.h>

#include <behaviortree_cpp/bt_factory.h>
#include <experimental_behaviors/shared_resources_node.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <pluginlib/class_loader.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/node.hpp>

#include <array>
#include <cstdint>
#include <set>
#include <string>
#include <tuple>
#include <vector>

namespace
{
constexpr auto kExcludedBehaviorsParameter = "experimental_behaviors.excluded_behaviors";
const std::array<const char*, 13> kBehaviorIDs = { "GetPoseStampedFromTopic",
                                                   "CreateDynamicInterfaceGroupValues",
                                                   "PublishDynamicInterfaceGroupValues",
                                                   "CreateInterfaceValue",
                                                   "GetDynamicInterfaceGroupValues",
                                                   "GetInterfaceValueFromGroup",
                                                   "AccessInterfaceValue",
                                                   "GetBlackboardByKey",
                                                   "GetJointLimits",
                                                   "SetBlackboardByKey",
                                                   "TrajectoryToPath",
                                                   "WriteYamlValue",
                                                   "AppendYamlListItem" };
const std::array<std::string, 5> kStagedIDs = { "CreateInterfaceValue", "CreateDynamicInterfaceGroupValues",
                                                "GetInterfaceValueFromGroup", "GetDynamicInterfaceGroupValues",
                                                "PublishDynamicInterfaceGroupValues" };

class ReplacementBehavior : public BT::SyncActionNode
{
public:
  using BT::SyncActionNode::SyncActionNode;

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override
  {
    return BT::NodeStatus::SUCCESS;
  }
};

std::set<std::string> registeredIDs(const BT::BehaviorTreeFactory& factory)
{
  std::set<std::string> ids;
  for (const auto& entry : factory.manifests())
  {
    ids.insert(entry.first);
  }
  return ids;
}

class BehaviorPlugins : public ::testing::Test
{
protected:
  void initialize(const std::vector<rclcpp::Parameter>& overrides = {})
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(overrides);
    node_ = std::make_shared<rclcpp::Node>("experimental_behavior_tests", options);
    shared_resources_ = std::make_shared<moveit_pro::behaviors::BehaviorContext>(node_, false);
  }

  void load(BT::BehaviorTreeFactory& factory)
  {
    auto plugin_instance = class_loader_.createUniqueInstance("experimental_behaviors::ExperimentalBehaviorsLoader");
    plugin_instance->registerBehaviors(factory, shared_resources_);
  }

  void expectAllBehaviorsAvailable()
  {
    EXPECT_EQ(factory_.manifests().size(), initial_count_ + kBehaviorIDs.size());
    for (const auto* id : kBehaviorIDs)
    {
      SCOPED_TRACE(id);
      ASSERT_EQ(factory_.manifests().count(id), 1u);
      BT::NodeConfiguration config;
      config.blackboard = BT::Blackboard::create();
      EXPECT_NO_THROW((void)factory_.instantiateTreeNode("test_behavior", id, config));
    }
  }

  void expectReplacement(const std::string& id)
  {
    BT::NodeConfiguration config;
    config.blackboard = BT::Blackboard::create();
    auto behavior = factory_.instantiateTreeNode("replacement", id, config);
    ASSERT_NE(dynamic_cast<ReplacementBehavior*>(behavior.get()), nullptr);
    EXPECT_EQ(behavior->executeTick(), BT::NodeStatus::SUCCESS);
  }

  pluginlib::ClassLoader<moveit_pro::behaviors::SharedResourcesNodeLoaderBase> class_loader_{
    "moveit_pro_behavior_interface", "moveit_pro::behaviors::SharedResourcesNodeLoaderBase"
  };
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<moveit_pro::behaviors::BehaviorContext> shared_resources_;
  BT::BehaviorTreeFactory factory_;
  const std::size_t initial_count_ = factory_.manifests().size();
};

TEST_F(BehaviorPlugins, DefaultRegistersAllThirteenBehaviors)
{
  initialize();
  ASSERT_NO_THROW(load(factory_));
  expectAllBehaviorsAvailable();
  EXPECT_TRUE(node_->describe_parameter(kExcludedBehaviorsParameter).read_only);
  EXPECT_EQ(node_->get_parameter(kExcludedBehaviorsParameter).get_type(), rclcpp::ParameterType::PARAMETER_STRING_ARRAY);
  EXPECT_TRUE(node_->get_parameter(kExcludedBehaviorsParameter).as_string_array().empty());
}

TEST_F(BehaviorPlugins, ExplicitEmptyArrayRegistersAllThirteenBehaviors)
{
  initialize({ rclcpp::Parameter(kExcludedBehaviorsParameter, std::vector<std::string>{}) });
  ASSERT_NO_THROW(load(factory_));
  expectAllBehaviorsAvailable();
}

TEST_F(BehaviorPlugins, ContextAdapterPreservesTheConstructorPointerForAllTenLeaves)
{
  initialize();
  ASSERT_NO_THROW(load(factory_));
  std::size_t adapter_count = 0;
  for (const auto* id : kBehaviorIDs)
  {
    BT::NodeConfiguration config;
    config.blackboard = BT::Blackboard::create();
    auto behavior = factory_.instantiateTreeNode("context_behavior", id, config);
    const auto* adapter =
        dynamic_cast<experimental_behaviors::ExperimentalSharedResourcesNode<BT::SyncActionNode>*>(behavior.get());
    if (adapter)
    {
      ++adapter_count;
      EXPECT_EQ(adapter->getExperimentalBehaviorContext(), shared_resources_) << id;
    }
  }
  EXPECT_EQ(adapter_count, 10u);
}

TEST_F(BehaviorPlugins, ExclusionsCannotBeChangedAfterRegistration)
{
  initialize();
  ASSERT_NO_THROW(load(factory_));
  const auto result = node_->set_parameter(
      rclcpp::Parameter(kExcludedBehaviorsParameter, std::vector<std::string>{ "CreateInterfaceValue" }));
  EXPECT_FALSE(result.successful);
  EXPECT_TRUE(node_->get_parameter(kExcludedBehaviorsParameter).as_string_array().empty());
  EXPECT_ANY_THROW(node_->undeclare_parameter(kExcludedBehaviorsParameter));
  expectAllBehaviorsAvailable();
}

TEST_F(BehaviorPlugins, ReusesTheReadOnlyParameterForAnotherFactory)
{
  initialize({ rclcpp::Parameter(kExcludedBehaviorsParameter, std::vector<std::string>{ "CreateInterfaceValue" }) });
  ASSERT_NO_THROW(load(factory_));
  BT::BehaviorTreeFactory second_factory;
  ASSERT_NO_THROW(load(second_factory));
  EXPECT_EQ(registeredIDs(factory_), registeredIDs(second_factory));
  EXPECT_EQ(factory_.manifests().count("CreateInterfaceValue"), 0u);
  EXPECT_EQ(factory_.manifests().size(), initial_count_ + 12u);
}

TEST_F(BehaviorPlugins, ValidExclusionsDoNotRequireReplacementsToBeLoadedYet)
{
  initialize({ rclcpp::Parameter(kExcludedBehaviorsParameter,
                                 std::vector<std::string>(kStagedIDs.begin(), kStagedIDs.end())) });
  ASSERT_NO_THROW(load(factory_));
  EXPECT_EQ(factory_.manifests().size(), initial_count_ + 8u);
  for (const auto& id : kStagedIDs)
  {
    EXPECT_EQ(factory_.manifests().count(id), 0u) << id;
  }
  const std::set<std::string> staged(kStagedIDs.begin(), kStagedIDs.end());
  for (const auto* id : kBehaviorIDs)
  {
    if (staged.count(id) == 0)
    {
      EXPECT_EQ(factory_.manifests().count(id), 1u) << id;
    }
  }
}

class SingleExclusion : public BehaviorPlugins, public ::testing::WithParamInterface<std::tuple<std::string, bool>>
{
};

TEST_P(SingleExclusion, PreservesReplacementInEitherLoaderOrder)
{
  const auto& [id, replacement_first] = GetParam();
  initialize({ rclcpp::Parameter(kExcludedBehaviorsParameter, std::vector<std::string>{ id }) });
  if (replacement_first)
  {
    factory_.registerNodeType<ReplacementBehavior>(id);
  }
  ASSERT_NO_THROW(load(factory_));
  if (!replacement_first)
  {
    ASSERT_EQ(factory_.manifests().count(id), 0u);
    factory_.registerNodeType<ReplacementBehavior>(id);
  }
  expectAllBehaviorsAvailable();
  expectReplacement(id);
}

TEST_P(SingleExclusion, UnconfiguredCollisionFailsInEitherLoaderOrder)
{
  const auto& [id, replacement_first] = GetParam();
  initialize();
  if (replacement_first)
  {
    factory_.registerNodeType<ReplacementBehavior>(id);
    EXPECT_ANY_THROW(load(factory_));
    expectReplacement(id);
  }
  else
  {
    ASSERT_NO_THROW(load(factory_));
    EXPECT_ANY_THROW(factory_.registerNodeType<ReplacementBehavior>(id));
    expectAllBehaviorsAvailable();
    BT::NodeConfiguration config;
    config.blackboard = BT::Blackboard::create();
    auto behavior = factory_.instantiateTreeNode("original_behavior", id, config);
    EXPECT_EQ(dynamic_cast<ReplacementBehavior*>(behavior.get()), nullptr);
  }
}

INSTANTIATE_TEST_SUITE_P(StagedBehaviorIDs, SingleExclusion,
                         ::testing::Combine(::testing::ValuesIn(kStagedIDs), ::testing::Bool()));

class AllExclusions : public BehaviorPlugins, public ::testing::WithParamInterface<bool>
{
};

TEST_P(AllExclusions, PreservesFiveReplacementsInEitherLoaderOrder)
{
  const bool replacement_first = GetParam();
  // Configuration order differs from registration order.
  initialize({ rclcpp::Parameter(kExcludedBehaviorsParameter,
                                 std::vector<std::string>(kStagedIDs.rbegin(), kStagedIDs.rend())) });
  if (replacement_first)
  {
    for (const auto& id : kStagedIDs)
    {
      factory_.registerNodeType<ReplacementBehavior>(id);
    }
  }
  ASSERT_NO_THROW(load(factory_));
  if (!replacement_first)
  {
    for (const auto& id : kStagedIDs)
    {
      ASSERT_EQ(factory_.manifests().count(id), 0u);
      factory_.registerNodeType<ReplacementBehavior>(id);
    }
  }
  expectAllBehaviorsAvailable();
  for (const auto& id : kStagedIDs)
  {
    expectReplacement(id);
  }
}

INSTANTIATE_TEST_SUITE_P(LoaderOrder, AllExclusions, ::testing::Bool());

TEST_F(BehaviorPlugins, ExclusionDoesNotSuppressAnotherUnconfiguredCollision)
{
  initialize({ rclcpp::Parameter(kExcludedBehaviorsParameter, std::vector<std::string>{ "CreateInterfaceValue" }) });
  factory_.registerNodeType<ReplacementBehavior>("GetInterfaceValueFromGroup");
  EXPECT_ANY_THROW(load(factory_));
  EXPECT_EQ(factory_.manifests().count("CreateInterfaceValue"), 0u);
  expectReplacement("GetInterfaceValueFromGroup");
}

class InvalidExclusion : public BehaviorPlugins, public ::testing::WithParamInterface<std::vector<std::string>>
{
};

TEST_P(InvalidExclusion, RejectsTheWholeListWithoutChangingTheFactory)
{
  initialize({ rclcpp::Parameter(kExcludedBehaviorsParameter, GetParam()) });
  factory_.registerNodeType<ReplacementBehavior>("ExistingReplacement");
  const auto before = registeredIDs(factory_);
  const auto builders_before = factory_.builders().size();
  EXPECT_THROW(load(factory_), BT::RuntimeError);
  EXPECT_EQ(registeredIDs(factory_), before);
  EXPECT_EQ(factory_.builders().size(), builders_before);
  EXPECT_FALSE(node_->has_parameter(kExcludedBehaviorsParameter));
  expectReplacement("ExistingReplacement");
}

INSTANTIATE_TEST_SUITE_P(
    InvalidIDs, InvalidExclusion,
    ::testing::Values(std::vector<std::string>{ "UnknownBehavior" }, std::vector<std::string>{ "" },
                      std::vector<std::string>{ "CreateInterfaceValue", "UnknownBehavior" },
                      std::vector<std::string>{ "CreateInterfaceValue", "" },
                      std::vector<std::string>{ "CreateInterfaceValue", "CreateInterfaceValue" },
                      std::vector<std::string>{ "createInterfaceValue" },
                      std::vector<std::string>{ " CreateInterfaceValue" },
                      std::vector<std::string>{ "GetPoseStampedFromTopic" },
                      std::vector<std::string>{ "AccessInterfaceValue" },
                      std::vector<std::string>{ "GetBlackboardByKey" }, std::vector<std::string>{ "GetJointLimits" },
                      std::vector<std::string>{ "SetBlackboardByKey" }, std::vector<std::string>{ "TrajectoryToPath" },
                      std::vector<std::string>{ "WriteYamlValue" }, std::vector<std::string>{ "AppendYamlListItem" }));

TEST_F(BehaviorPlugins, WrongParameterTypesFailBeforeRegistration)
{
  for (const auto& value :
       { rclcpp::Parameter(kExcludedBehaviorsParameter, "CreateInterfaceValue"),
         rclcpp::Parameter(kExcludedBehaviorsParameter, true), rclcpp::Parameter(kExcludedBehaviorsParameter, 1),
         rclcpp::Parameter(kExcludedBehaviorsParameter, 1.0),
         rclcpp::Parameter(kExcludedBehaviorsParameter, std::vector<std::int64_t>{ 1 }),
         rclcpp::Parameter(kExcludedBehaviorsParameter, std::vector<double>{ 1.0 }),
         rclcpp::Parameter(kExcludedBehaviorsParameter, std::vector<bool>{ true }) })
  {
    SCOPED_TRACE(value.get_type_name());
    initialize({ value });
    const auto before = registeredIDs(factory_);
    EXPECT_THROW(load(factory_), BT::RuntimeError);
    EXPECT_EQ(registeredIDs(factory_), before);
    EXPECT_FALSE(node_->has_parameter(kExcludedBehaviorsParameter));
  }
}

TEST_F(BehaviorPlugins, PredeclaredWritableParameterIsRejected)
{
  initialize();
  node_->declare_parameter<std::vector<std::string>>(kExcludedBehaviorsParameter, std::vector<std::string>{});
  const auto before = registeredIDs(factory_);
  EXPECT_THROW(load(factory_), BT::RuntimeError);
  EXPECT_EQ(registeredIDs(factory_), before);
}

TEST_F(BehaviorPlugins, PredeclaredReadOnlyStringArrayIsAccepted)
{
  initialize();
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  node_->declare_parameter<std::vector<std::string>>(kExcludedBehaviorsParameter,
                                                     std::vector<std::string>{ "CreateInterfaceValue" }, descriptor);
  ASSERT_NO_THROW(load(factory_));
  EXPECT_EQ(factory_.manifests().count("CreateInterfaceValue"), 0u);
  EXPECT_EQ(factory_.manifests().size(), initial_count_ + 12u);
}
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
