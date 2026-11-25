#include "grasping_behaviors/extract_graspable_object_primitive.hpp"

#include <spdlog/spdlog.h>
#include <moveit_studio_vision_msgs/msg/graspable_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

namespace
{
constexpr auto kPortIDGraspableObject = "graspable_object";
constexpr auto kPortIDSolidPrimitive = "solid_primitive";

inline constexpr auto kDescription = R"(
    <p>
        Pulls the first <b>shape_msgs::msg::SolidPrimitive</b> out of a
        <b>moveit_studio_vision_msgs::msg::GraspableObject</b>'s underlying collision object.
        Fails if the object has no primitives.
    </p>
)";

}  // namespace

namespace grasping_behaviors
{

ExtractGraspableObjectPrimitive::ExtractGraspableObjectPrimitive(const std::string& name,
                                                                 const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config)
{
}

BT::PortsList ExtractGraspableObjectPrimitive::providedPorts()
{
  return {
    BT::InputPort<moveit_studio_vision_msgs::msg::GraspableObject>(kPortIDGraspableObject, "{graspable_object}",
                                                                   "GraspableObject to extract the primitive from."),
    BT::OutputPort<shape_msgs::msg::SolidPrimitive>(
        kPortIDSolidPrimitive, "{solid_primitive}", "First SolidPrimitive from the GraspableObject's collision object."),
  };
}

BT::KeyValueVector ExtractGraspableObjectPrimitive::metadata()
{
  return { { "subcategory", "Grasping" }, { "description", kDescription } };
}

BT::NodeStatus ExtractGraspableObjectPrimitive::tick()
{
  const auto graspable_object_input = getInput<moveit_studio_vision_msgs::msg::GraspableObject>(kPortIDGraspableObject);
  if (!graspable_object_input.has_value())
  {
    spdlog::error("ExtractGraspableObjectPrimitive: failed to get 'graspable_object' input port.");
    return BT::NodeStatus::FAILURE;
  }

  const auto& primitives = graspable_object_input.value().object.primitives;
  if (primitives.empty())
  {
    spdlog::error("ExtractGraspableObjectPrimitive: graspable_object has no primitives.");
    return BT::NodeStatus::FAILURE;
  }

  setOutput(kPortIDSolidPrimitive, primitives.front());
  return BT::NodeStatus::SUCCESS;
}

}  // namespace grasping_behaviors
