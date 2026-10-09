// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "json_behavior_test_fixture.hpp"

#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/contrib/json.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_pro_behavior_interface/json_serialization.hpp>

#include <cstdint>
#include <optional>

using nlohmann::json;

namespace experimental_behaviors::test
{
namespace
{
/// A type with no JSON converter, to exercise the "cannot serialize" failure.
struct OpaqueThing
{
  int value = 0;
};

geometry_msgs::msg::PoseStamped makePose()
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "world";
  pose.header.stamp.sec = 12;
  pose.header.stamp.nanosec = 34;
  pose.pose.position.x = 0.5;
  pose.pose.position.y = -1.25;
  pose.pose.position.z = 2.0;
  pose.pose.orientation.z = 0.7071067811865476;
  pose.pose.orientation.w = 0.7071067811865476;
  return pose;
}
}  // namespace

class JsonEditingTest : public JsonBehaviorTest
{
protected:
  static void SetUpTestSuite()
  {
    // The Objective Server registers these converters at startup; the UI blackboard view uses them.
    register_ros_msg<geometry_msgs::msg::PoseStamped>();
  }

  json docAt(const std::string& key)
  {
    return json::parse(blackboard_->get<std::string>(key));
  }
};

TEST_F(JsonEditingTest, CreateJsonDefaultsToEmptyObject)
{
  ASSERT_EQ(run(R"(<CreateJson json="{doc}"/>)"), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(blackboard_->get<std::string>("doc"), "{}");
}

TEST_F(JsonEditingTest, CreateJsonCopiesTemplateTypedIntoThePort)
{
  // The literal starts with '{' and ends with '}', which BT.CPP would otherwise read as a blackboard key.
  ASSERT_EQ(run(R"(<CreateJson initial='{"robot": "arm", "items": [1, 2]}' json="{doc}"/>)"), BT::NodeStatus::SUCCESS)
      << errors();
  EXPECT_EQ(docAt("doc"), json::parse(R"({"robot": "arm", "items": [1, 2]})"));
}

TEST_F(JsonEditingTest, CreateJsonCopiesTemplateFromBlackboard)
{
  blackboard_->set<std::string>("template", R"([1, "two", null])");
  ASSERT_EQ(run(R"(<CreateJson initial="{template}" json="{doc}"/>)"), BT::NodeStatus::SUCCESS) << errors();
  EXPECT_EQ(docAt("doc"), json::parse(R"([1, "two", null])"));
}

TEST_F(JsonEditingTest, CreateJsonRejectsInvalidJsonWithLocation)
{
  blackboard_->set<std::string>("template", "{\"a\": 1,\n \"b\": }");
  EXPECT_EQ(run(R"(<CreateJson initial="{template}" json="{doc}"/>)"), BT::NodeStatus::FAILURE);
  const auto message = errors();
  EXPECT_NE(message.find("[initial] is not valid JSON"), std::string::npos) << message;
  EXPECT_NE(message.find("line 2, column"), std::string::npos) << message;
}

TEST_F(JsonEditingTest, SetAndGetRoundTripString)
{
  blackboard_->set<std::string>("doc", "{}");
  blackboard_->set<std::string>("in", "hello");
  ASSERT_EQ(run(R"(<Sequence>
                     <SetJsonField json="{doc}" path="/greeting" value="{in}"/>
                     <GetJsonField json="{doc}" path="/greeting" value="{out}"/>
                   </Sequence>)"),
            BT::NodeStatus::SUCCESS)
      << errors();
  EXPECT_EQ(docAt("doc"), json::parse(R"({"greeting": "hello"})"));
  EXPECT_EQ(blackboard_->get<std::string>("out"), "hello");
}

TEST_F(JsonEditingTest, SetAndGetRoundTripInteger)
{
  blackboard_->set<std::string>("doc", "{}");
  blackboard_->set<int>("in", -42);
  ASSERT_EQ(run(R"(<Sequence>
                     <SetJsonField json="{doc}" path="/count" value="{in}"/>
                     <GetJsonField json="{doc}" path="/count" value="{out}"/>
                   </Sequence>)"),
            BT::NodeStatus::SUCCESS)
      << errors();
  EXPECT_TRUE(docAt("doc")["count"].is_number_integer());
  EXPECT_EQ(blackboard_->get<int>("out"), -42);
}

TEST_F(JsonEditingTest, SetAndGetRoundTripUnsigned)
{
  blackboard_->set<std::string>("doc", "{}");
  blackboard_->set<uint64_t>("in", 18446744073709551615ULL);
  ASSERT_EQ(run(R"(<Sequence>
                     <SetJsonField json="{doc}" path="/big" value="{in}"/>
                     <GetJsonField json="{doc}" path="/big" value="{out}"/>
                   </Sequence>)"),
            BT::NodeStatus::SUCCESS)
      << errors();
  EXPECT_EQ(docAt("doc")["big"].get<uint64_t>(), 18446744073709551615ULL);
  EXPECT_EQ(blackboard_->get<uint64_t>("out"), 18446744073709551615ULL);
}

TEST_F(JsonEditingTest, SetAndGetRoundTripDouble)
{
  blackboard_->set<std::string>("doc", "{}");
  blackboard_->set<double>("in", 0.1);
  ASSERT_EQ(run(R"(<Sequence>
                     <SetJsonField json="{doc}" path="/ratio" value="{in}"/>
                     <GetJsonField json="{doc}" path="/ratio" value="{out}"/>
                   </Sequence>)"),
            BT::NodeStatus::SUCCESS)
      << errors();
  EXPECT_TRUE(docAt("doc")["ratio"].is_number_float());
  EXPECT_DOUBLE_EQ(blackboard_->get<double>("out"), 0.1);
}

TEST_F(JsonEditingTest, SetAndGetRoundTripBool)
{
  blackboard_->set<std::string>("doc", "{}");
  blackboard_->set<bool>("in", true);
  ASSERT_EQ(run(R"(<Sequence>
                     <SetJsonField json="{doc}" path="/ok" value="{in}"/>
                     <GetJsonField json="{doc}" path="/ok" value="{out}"/>
                   </Sequence>)"),
            BT::NodeStatus::SUCCESS)
      << errors();
  EXPECT_TRUE(docAt("doc")["ok"].is_boolean());
  EXPECT_TRUE(blackboard_->get<bool>("out"));
}

TEST_F(JsonEditingTest, SetAndGetRoundTripPoseStamped)
{
  blackboard_->set<std::string>("doc", "{}");
  blackboard_->set("in", makePose());
  ASSERT_EQ(run(R"(<Sequence>
                     <SetJsonField json="{doc}" path="/target/pose" value="{in}"/>
                     <GetJsonField json="{doc}" path="/target/pose" message_type="geometry_msgs/msg/PoseStamped"
                                   value="{out}"/>
                   </Sequence>)"),
            BT::NodeStatus::SUCCESS)
      << errors();
  // Pro's converter prints six significant digits, so compare at that precision.
  const auto out = blackboard_->get<geometry_msgs::msg::PoseStamped>("out");
  const auto expected = makePose();
  EXPECT_EQ(out.header, expected.header);
  EXPECT_NEAR(out.pose.position.x, expected.pose.position.x, 1e-6);
  EXPECT_NEAR(out.pose.position.y, expected.pose.position.y, 1e-6);
  EXPECT_NEAR(out.pose.position.z, expected.pose.position.z, 1e-6);
  EXPECT_NEAR(out.pose.orientation.x, expected.pose.orientation.x, 1e-6);
  EXPECT_NEAR(out.pose.orientation.y, expected.pose.orientation.y, 1e-6);
  EXPECT_NEAR(out.pose.orientation.z, expected.pose.orientation.z, 1e-6);
  EXPECT_NEAR(out.pose.orientation.w, expected.pose.orientation.w, 1e-6);
}

TEST_F(JsonEditingTest, SetPoseStampedMatchesUiBlackboardView)
{
  // The UI blackboard view is built with BT::ExportBlackboardToJSON.
  blackboard_->set<std::string>("doc", "{}");
  blackboard_->set("pose", makePose());
  ASSERT_EQ(run(R"(<SetJsonField json="{doc}" path="/pose" value="{pose}"/>)"), BT::NodeStatus::SUCCESS) << errors();
  const json ui_view = BT::ExportBlackboardToJSON(*blackboard_);
  EXPECT_EQ(docAt("doc")["pose"], ui_view["pose"]);
  EXPECT_EQ(docAt("doc")["pose"]["__type"], "geometry_msgs::msg::PoseStamped");
}

TEST_F(JsonEditingTest, SetValueThroughSubTreeRemapping)
{
  // The raw [value] port must resolve a key remapped into a SubTree, as getInput would.
  blackboard_->set<std::string>("doc", "{}");
  blackboard_->set<double>("outer_value", 1.5);
  const std::string xml = R"(<root BTCPP_format="4" main_tree_to_execute="Main">
      <BehaviorTree ID="Main"><SubTree ID="Inner" document="{doc}" number="{outer_value}"/></BehaviorTree>
      <BehaviorTree ID="Inner"><SetJsonField json="{document}" path="/n" value="{number}"/></BehaviorTree>
    </root>)";
  tree_ = factory_.createTreeFromText(xml, blackboard_);
  ASSERT_EQ(tree_.tickWhileRunning(), BT::NodeStatus::SUCCESS) << errors();
  EXPECT_EQ(docAt("doc"), json::parse(R"({"n": 1.5})"));
}

TEST_F(JsonEditingTest, GetPoseStampedFromPlainNumbers)
{
  blackboard_->set<std::string>("doc", R"({"goal": {"header": {"frame_id": "map"},
                          "pose": {"position": {"x": 1, "y": 2.5, "z": 0},
                                   "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}}}})");
  ASSERT_EQ(run(R"(<GetJsonField json="{doc}" path="/goal" message_type="geometry_msgs::msg::PoseStamped"
                                 value="{out}"/>)"),
            BT::NodeStatus::SUCCESS)
      << errors();
  const auto out = blackboard_->get<geometry_msgs::msg::PoseStamped>("out");
  EXPECT_EQ(out.header.frame_id, "map");
  EXPECT_DOUBLE_EQ(out.pose.position.y, 2.5);
  EXPECT_DOUBLE_EQ(out.pose.orientation.w, 1.0);
}

TEST_F(JsonEditingTest, GetScalarsFeedTypedPorts)
{
  std::optional<double> as_double;
  std::optional<int> as_int;
  std::optional<bool> as_bool;
  std::optional<std::string> as_string;
  factory_.registerSimpleAction("ReadDouble",
                                [&](BT::TreeNode& node) {
                                  auto v = node.getInput<double>("in");
                                  as_double = v ? std::optional(*v) : std::nullopt;
                                  return BT::NodeStatus::SUCCESS;
                                },
                                { BT::InputPort<double>("in") });
  factory_.registerSimpleAction("ReadInt",
                                [&](BT::TreeNode& node) {
                                  auto v = node.getInput<int>("in");
                                  as_int = v ? std::optional(*v) : std::nullopt;
                                  return BT::NodeStatus::SUCCESS;
                                },
                                { BT::InputPort<int>("in") });
  factory_.registerSimpleAction("ReadBool",
                                [&](BT::TreeNode& node) {
                                  auto v = node.getInput<bool>("in");
                                  as_bool = v ? std::optional(*v) : std::nullopt;
                                  return BT::NodeStatus::SUCCESS;
                                },
                                { BT::InputPort<bool>("in") });
  factory_.registerSimpleAction("ReadString",
                                [&](BT::TreeNode& node) {
                                  auto v = node.getInput<std::string>("in");
                                  as_string = v ? std::optional(*v) : std::nullopt;
                                  return BT::NodeStatus::SUCCESS;
                                },
                                { BT::InputPort<std::string>("in") });

  blackboard_->set<std::string>("doc", R"({"speed": 0.25, "count": 3, "ok": true, "name": "arm"})");
  ASSERT_EQ(run(R"(<Sequence>
                     <GetJsonField json="{doc}" path="/speed" value="{speed}"/>
                     <ReadDouble in="{speed}"/>
                     <GetJsonField json="{doc}" path="/count" value="{count}"/>
                     <ReadInt in="{count}"/>
                     <GetJsonField json="{doc}" path="/ok" value="{ok}"/>
                     <ReadBool in="{ok}"/>
                     <GetJsonField json="{doc}" path="/name" value="{name}"/>
                     <ReadString in="{name}"/>
                     <Script code="sum := count + speed"/>
                   </Sequence>)"),
            BT::NodeStatus::SUCCESS)
      << errors();
  EXPECT_EQ(as_double, 0.25);
  EXPECT_EQ(as_int, 3);
  EXPECT_EQ(as_bool, true);
  EXPECT_EQ(as_string, "arm");
  EXPECT_DOUBLE_EQ(blackboard_->get<double>("sum"), 3.25);
}

TEST_F(JsonEditingTest, GetObjectComesOutAsJsonText)
{
  blackboard_->set<std::string>("doc", R"({"status": {"state": "idle", "codes": [1, 2]}})");
  ASSERT_EQ(run(R"(<GetJsonField json="{doc}" path="/status" value="{out}"/>)"), BT::NodeStatus::SUCCESS) << errors();
  EXPECT_EQ(json::parse(blackboard_->get<std::string>("out")), json::parse(R"({"state": "idle", "codes": [1, 2]})"));
}

TEST_F(JsonEditingTest, LiteralValueIsStringUnlessParsedAsJson)
{
  blackboard_->set<std::string>("doc", "{}");
  ASSERT_EQ(run(R"(<Sequence>
                     <SetJsonField json="{doc}" path="/as_string" value="42"/>
                     <SetJsonField json="{doc}" path="/as_number" value="42" parse_value_as_json="true"/>
                     <SetJsonField json="{doc}" path="/as_object" value='{"a": [true]}' parse_value_as_json="true"/>
                     <SetJsonField json="{doc}" path="/object_text" value='{"a": [true]}'/>
                   </Sequence>)"),
            BT::NodeStatus::SUCCESS)
      << errors();
  const json doc = docAt("doc");
  EXPECT_EQ(doc["as_string"], "42");
  EXPECT_EQ(doc["as_number"], 42);
  EXPECT_EQ(doc["as_object"], json::parse(R"({"a": [true]})"));
  EXPECT_EQ(doc["object_text"], R"({"a": [true]})");
}

TEST_F(JsonEditingTest, ParseValueAsJsonReportsParseError)
{
  blackboard_->set<std::string>("doc", "{}");
  EXPECT_EQ(run(R"(<SetJsonField json="{doc}" path="/a" value="[1, 2" parse_value_as_json="true"/>)"),
            BT::NodeStatus::FAILURE);
  const auto message = errors();
  EXPECT_NE(message.find("[value] is not valid JSON"), std::string::npos) << message;
  EXPECT_NE(message.find("line 1, column"), std::string::npos) << message;
}

TEST_F(JsonEditingTest, SetCreatesNestedParents)
{
  blackboard_->set<std::string>("doc", "{}");
  ASSERT_EQ(run(R"(<SetJsonField json="{doc}" path="/a/b/c" value="deep"/>)"), BT::NodeStatus::SUCCESS) << errors();
  EXPECT_EQ(docAt("doc"), json::parse(R"({"a": {"b": {"c": "deep"}}})"));
}

TEST_F(JsonEditingTest, SetWithoutCreateMissingFailsOnMissingParent)
{
  blackboard_->set<std::string>("doc", R"({"a": {}})");
  EXPECT_EQ(run(R"(<SetJsonField json="{doc}" path="/a/b/c" value="x" create_missing="false"/>)"),
            BT::NodeStatus::FAILURE);
  const auto message = errors();
  EXPECT_NE(message.find("'/a/b' does not exist"), std::string::npos) << message;
  EXPECT_EQ(docAt("doc"), json::parse(R"({"a": {}})"));

  // The leaf itself may be new even without create_missing.
  ASSERT_EQ(run(R"(<SetJsonField json="{doc}" path="/a/b" value="x" create_missing="false"/>)"), BT::NodeStatus::SUCCESS)
      << errors();
}

TEST_F(JsonEditingTest, SetArrayElements)
{
  blackboard_->set<std::string>("doc", R"({"items": [1, 2]})");
  ASSERT_EQ(run(R"(<Sequence>
                     <SetJsonField json="{doc}" path="/items/0" value="10" parse_value_as_json="true"/>
                     <SetJsonField json="{doc}" path="/items/2" value="3" parse_value_as_json="true"/>
                     <SetJsonField json="{doc}" path="/items/-" value="4" parse_value_as_json="true"/>
                   </Sequence>)"),
            BT::NodeStatus::SUCCESS)
      << errors();
  EXPECT_EQ(docAt("doc"), json::parse(R"({"items": [10, 2, 3, 4]})"));

  EXPECT_EQ(run(R"(<SetJsonField json="{doc}" path="/items/9" value="x"/>)"), BT::NodeStatus::FAILURE);
  EXPECT_NE(errors().find("past the end of the array at '/items'"), std::string::npos);
  EXPECT_EQ(run(R"(<SetJsonField json="{doc}" path="/items/01" value="x"/>)"), BT::NodeStatus::FAILURE);
  EXPECT_NE(errors().find("not a valid array index"), std::string::npos);
}

TEST_F(JsonEditingTest, EscapedPathTokens)
{
  blackboard_->set<std::string>("doc", "{}");
  ASSERT_EQ(run(R"(<Sequence>
                     <SetJsonField json="{doc}" path="/a~1b/c~0d" value="v"/>
                     <GetJsonField json="{doc}" path="/a~1b/c~0d" value="{out}"/>
                   </Sequence>)"),
            BT::NodeStatus::SUCCESS)
      << errors();
  EXPECT_EQ(docAt("doc"), json::parse(R"({"a/b": {"c~d": "v"}})"));
  EXPECT_EQ(blackboard_->get<std::string>("out"), "v");
}

TEST_F(JsonEditingTest, SetEmptyPathReplacesDocument)
{
  blackboard_->set<std::string>("doc", R"({"old": 1})");
  ASSERT_EQ(run(R"(<SetJsonField json="{doc}" path="" value="[1]" parse_value_as_json="true"/>)"),
            BT::NodeStatus::SUCCESS)
      << errors();
  EXPECT_EQ(docAt("doc"), json::parse("[1]"));
}

TEST_F(JsonEditingTest, SetIntoScalarFails)
{
  blackboard_->set<std::string>("doc", R"({"a": 5})");
  EXPECT_EQ(run(R"(<SetJsonField json="{doc}" path="/a/b" value="x"/>)"), BT::NodeStatus::FAILURE);
  const auto message = errors();
  EXPECT_NE(message.find("'/a' is a number, which cannot hold fields"), std::string::npos) << message;
}

TEST_F(JsonEditingTest, SetUnserializableValueNamesTheType)
{
  blackboard_->set<std::string>("doc", "{}");
  blackboard_->set("thing", OpaqueThing{});
  EXPECT_EQ(run(R"(<SetJsonField json="{doc}" path="/thing" value="{thing}"/>)"), BT::NodeStatus::FAILURE);
  const auto message = errors();
  EXPECT_NE(message.find("cannot serialize a value of C++ type"), std::string::npos) << message;
  EXPECT_NE(message.find("OpaqueThing"), std::string::npos) << message;
}

TEST_F(JsonEditingTest, SetRejectsInvalidDocumentAndPath)
{
  blackboard_->set<std::string>("doc", "not json");
  EXPECT_EQ(run(R"(<SetJsonField json="{doc}" path="/a" value="x"/>)"), BT::NodeStatus::FAILURE);
  auto message = errors();
  EXPECT_NE(message.find("[json] is not valid JSON"), std::string::npos) << message;
  EXPECT_NE(message.find("line 1, column 2"), std::string::npos) << message;

  blackboard_->set<std::string>("doc", "{}");
  EXPECT_EQ(run(R"(<SetJsonField json="{doc}" path="a" value="x"/>)"), BT::NodeStatus::FAILURE);
  message = errors();
  EXPECT_NE(message.find("not a valid JSON Pointer"), std::string::npos) << message;

  EXPECT_EQ(run(R"(<SetJsonField json="{doc}" path="/a~2" value="x"/>)"), BT::NodeStatus::FAILURE);
  message = errors();
  EXPECT_NE(message.find("not a valid JSON Pointer"), std::string::npos) << message;
}

TEST_F(JsonEditingTest, GetMissingPathFails)
{
  blackboard_->set<std::string>("doc", R"({"a": {"b": [1]}})");
  EXPECT_EQ(run(R"(<GetJsonField json="{doc}" path="/a/c" value="{out}"/>)"), BT::NodeStatus::FAILURE);
  auto message = errors();
  EXPECT_NE(message.find("no field at '/a/c'"), std::string::npos) << message;

  EXPECT_EQ(run(R"(<GetJsonField json="{doc}" path="/a/b/1" value="{out}"/>)"), BT::NodeStatus::FAILURE);
  message = errors();
  EXPECT_NE(message.find("has 1 elements"), std::string::npos) << message;

  ASSERT_EQ(run(R"(<GetJsonField json="{doc}" path="/a/b/0" value="{out}"/>)"), BT::NodeStatus::SUCCESS) << errors();
  EXPECT_EQ(blackboard_->get<int>("out"), 1);
}

TEST_F(JsonEditingTest, GetMessageTypeMismatchFails)
{
  blackboard_->set<std::string>("doc", R"({"n": 3, "bad": {"pose": {"position": {"x": "abc", "y": 0, "z": 0},
                                          "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}}}})");
  EXPECT_EQ(run(R"(<GetJsonField json="{doc}" path="/n" message_type="geometry_msgs/msg/PoseStamped" value="{out}"/>)"),
            BT::NodeStatus::FAILURE);
  auto message = errors();
  EXPECT_NE(message.find("'/n' is a number, expected an object"), std::string::npos) << message;

  EXPECT_EQ(
      run(R"(<GetJsonField json="{doc}" path="/bad" message_type="geometry_msgs/msg/PoseStamped" value="{out}"/>)"),
      BT::NodeStatus::FAILURE);
  message = errors();
  EXPECT_NE(message.find(R"('/bad/pose/position/x' is the string "abc", expected a number)"), std::string::npos)
      << message;

  EXPECT_EQ(run(R"(<GetJsonField json="{doc}" path="/n" message_type="std_msgs/msg/Bogus" value="{out}"/>)"),
            BT::NodeStatus::FAILURE);
  message = errors();
  EXPECT_NE(message.find("unsupported message_type"), std::string::npos) << message;
}

TEST_F(JsonEditingTest, GetMessageTypeChecksTypeTag)
{
  blackboard_->set<std::string>("doc", "{}");
  blackboard_->set("pose", makePose());
  EXPECT_EQ(run(R"(<Sequence>
                     <SetJsonField json="{doc}" path="/p" value="{pose}"/>
                     <GetJsonField json="{doc}" path="/p" message_type="geometry_msgs/msg/Pose" value="{out}"/>
                   </Sequence>)"),
            BT::NodeStatus::FAILURE);
  const auto message = errors();
  EXPECT_NE(message.find("tagged as 'geometry_msgs::msg::PoseStamped'"), std::string::npos) << message;
}

TEST_F(JsonEditingTest, RemoveField)
{
  blackboard_->set<std::string>("doc", R"({"a": 1, "items": [1, 2, 3]})");
  ASSERT_EQ(run(R"(<Sequence>
                     <RemoveJsonField json="{doc}" path="/a"/>
                     <RemoveJsonField json="{doc}" path="/items/0"/>
                   </Sequence>)"),
            BT::NodeStatus::SUCCESS)
      << errors();
  EXPECT_EQ(docAt("doc"), json::parse(R"({"items": [2, 3]})"));
}

TEST_F(JsonEditingTest, RemoveMissingFieldSucceedsWithoutChange)
{
  const std::string original = R"({"a": {"b": 1}})";
  blackboard_->set<std::string>("doc", original);
  ASSERT_EQ(run(R"(<Sequence>
                     <RemoveJsonField json="{doc}" path="/x"/>
                     <RemoveJsonField json="{doc}" path="/a/b/c"/>
                     <RemoveJsonField json="{doc}" path="/x/y"/>
                   </Sequence>)"),
            BT::NodeStatus::SUCCESS)
      << errors();
  EXPECT_EQ(blackboard_->get<std::string>("doc"), original);
}

TEST_F(JsonEditingTest, RemoveRootFails)
{
  blackboard_->set<std::string>("doc", "{}");
  EXPECT_EQ(run(R"(<RemoveJsonField json="{doc}" path=""/>)"), BT::NodeStatus::FAILURE);
  EXPECT_NE(errors().find("cannot remove the document root"), std::string::npos);
}

TEST_F(JsonEditingTest, HasField)
{
  blackboard_->set<std::string>("doc", R"({"a": null, "items": [0], "k/x": 1})");
  EXPECT_EQ(run(R"(<HasJsonField json="{doc}" path="/a"/>)"), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(run(R"(<HasJsonField json="{doc}" path="/items/0"/>)"), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(run(R"(<HasJsonField json="{doc}" path="/k~1x"/>)"), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(run(R"(<HasJsonField json="{doc}" path=""/>)"), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(run(R"(<HasJsonField json="{doc}" path="/b"/>)"), BT::NodeStatus::FAILURE);
  EXPECT_EQ(run(R"(<HasJsonField json="{doc}" path="/items/1"/>)"), BT::NodeStatus::FAILURE);
  EXPECT_EQ(run(R"(<HasJsonField json="{doc}" path="/a/b"/>)"), BT::NodeStatus::FAILURE);
  // A missing field is a plain FAILURE, not an error.
  EXPECT_EQ(errors(), "");

  EXPECT_EQ(run(R"(<HasJsonField json="{doc}" path="b"/>)"), BT::NodeStatus::FAILURE);
  EXPECT_NE(errors().find("not a valid JSON Pointer"), std::string::npos);
}

TEST_F(JsonEditingTest, MergePatchFollowsRfc7386)
{
  // Example from RFC 7386 section 3.
  blackboard_->set<std::string>("doc", R"({"title": "Goodbye!", "author": {"givenName": "John", "familyName": "Doe"},
                                           "tags": ["example", "sample"], "content": "This will be unchanged"})");
  blackboard_->set<std::string>("patch", R"({"title": "Hello!", "phoneNumber": "+01-123-456-7890",
                                             "author": {"familyName": null}, "tags": ["example"]})");
  ASSERT_EQ(run(R"(<MergeJson json="{doc}" patch="{patch}"/>)"), BT::NodeStatus::SUCCESS) << errors();
  EXPECT_EQ(docAt("doc"), json::parse(R"({"title": "Hello!", "author": {"givenName": "John"}, "tags": ["example"],
                                          "content": "This will be unchanged", "phoneNumber": "+01-123-456-7890"})"));
}

TEST_F(JsonEditingTest, MergePatchTypedIntoThePort)
{
  blackboard_->set<std::string>("doc", R"({"a": 1, "b": 2})");
  ASSERT_EQ(run(R"(<MergeJson json="{doc}" patch='{"b": null, "c": {"d": true}}'/>)"), BT::NodeStatus::SUCCESS)
      << errors();
  EXPECT_EQ(docAt("doc"), json::parse(R"({"a": 1, "c": {"d": true}})"));
}

TEST_F(JsonEditingTest, MergeRejectsInvalidPatch)
{
  blackboard_->set<std::string>("doc", "{}");
  blackboard_->set<std::string>("patch", "{");
  EXPECT_EQ(run(R"(<MergeJson json="{doc}" patch="{patch}"/>)"), BT::NodeStatus::FAILURE);
  EXPECT_NE(errors().find("[patch] is not valid JSON"), std::string::npos);
}

}  // namespace experimental_behaviors::test

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
