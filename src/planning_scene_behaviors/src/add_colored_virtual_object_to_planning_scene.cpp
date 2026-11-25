// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "planning_scene_behaviors/add_colored_virtual_object_to_planning_scene.hpp"

#include <algorithm>
#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

#include <fmt/format.h>
#include <spdlog/spdlog.h>

#include <moveit_msgs/msg/allowed_collision_entry.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/object_color.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

namespace
{
inline constexpr auto kDescription = R"(
<p>
  Add a <code>moveit_msgs/CollisionObject</code> to the planning scene as a "virtual" obstacle
  with the specified RGBA color, in a single <code>/apply_planning_scene</code> call.
</p>
<p>
  Replaces the
  <code>Convert CollisionObject To GraspableObject</code> &rarr;
  <code>UnpackGraspableObjectMessage</code> &rarr;
  <code>CreateColorRGBAMessage</code> &rarr;
  <code>CreateObjectColorMessage</code> &rarr;
  <code>CreateGraspableObjectMessage</code> &rarr;
  <code>AddVirtualObjectToPlanningScene</code>
  six-step chain that's otherwise required to add a colored virtual obstacle.
</p>
<p>
  Mirrors the "virtual_" id-prefix and allowed-collision-matrix conventions of MoveIt Pro's
  <code>AddVirtualObjectToPlanningScene</code>: forbids collisions with all non-virtual objects,
  allows collisions with all other virtual objects. Requires a fresh
  <code>planning_scene_msg</code>, typically produced by a preceding
  <code>GetCurrentPlanningScene</code> call.
</p>
)";

constexpr auto kPortIdCollisionObject = "collision_object";
constexpr auto kPortIdPlanningSceneMsg = "planning_scene_msg";
constexpr auto kPortIdR = "r";
constexpr auto kPortIdG = "g";
constexpr auto kPortIdB = "b";
constexpr auto kPortIdA = "a";

constexpr double kDefaultR = 1.0;
constexpr double kDefaultG = 1.0;
constexpr double kDefaultB = 1.0;
constexpr double kDefaultA = 0.9;

constexpr auto kApplyPlanningSceneServiceName = "/apply_planning_scene";

constexpr std::string_view kVirtualPrefix = "virtual_";

// Returns true iff `s` begins with `kVirtualPrefix`. The original code used
// std::string::find(prefix) == npos, which matches the substring *anywhere* and so
// mis-classified ids like "my_virtual_box" as already-virtual.
[[nodiscard]] bool hasVirtualPrefix(std::string_view s)
{
  return s.compare(0, kVirtualPrefix.size(), kVirtualPrefix) == 0;
}
}  // namespace

namespace planning_scene_behaviors
{
AddColoredVirtualObjectToPlanningScene::AddColoredVirtualObjectToPlanningScene(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : ServiceClientBehaviorBase<ApplyPlanningScene>(name, config, shared_resources)
{
}

BT::PortsList AddColoredVirtualObjectToPlanningScene::providedPorts()
{
  return {
    BT::InputPort<moveit_msgs::msg::CollisionObject>(kPortIdCollisionObject, "{collision_object}",
                                                     "CollisionObject to add to the planning scene as a virtual "
                                                     "obstacle. Its `id` field is "
                                                     "prefixed with \"virtual_\" inside this behavior if not already."),
    BT::InputPort<moveit_msgs::msg::PlanningScene>(kPortIdPlanningSceneMsg, "{planning_scene}",
                                                   "Snapshot of the current planning scene to diff against. Typically "
                                                   "produced by an "
                                                   "immediately preceding GetCurrentPlanningScene call."),
    BT::InputPort<double>(kPortIdR, kDefaultR, "Red channel, 0-1. Defaults to 1.0 (white)."),
    BT::InputPort<double>(kPortIdG, kDefaultG, "Green channel, 0-1. Defaults to 1.0 (white)."),
    BT::InputPort<double>(kPortIdB, kDefaultB, "Blue channel, 0-1. Defaults to 1.0 (white)."),
    BT::InputPort<double>(kPortIdA, kDefaultA, "Alpha channel, 0-1. Defaults to 0.9."),
  };
}

BT::KeyValueVector AddColoredVirtualObjectToPlanningScene::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "Planning Scene" },
           { moveit_pro::behaviors::kDescriptionMetadataKey, kDescription } };
}

tl::expected<std::string, std::string> AddColoredVirtualObjectToPlanningScene::getServiceName()
{
  return std::string{ kApplyPlanningSceneServiceName };
}

tl::expected<ApplyPlanningScene::Request, std::string> AddColoredVirtualObjectToPlanningScene::createRequest()
{
  const auto required =
      moveit_pro::behaviors::getRequiredInputs(getInput<moveit_msgs::msg::CollisionObject>(kPortIdCollisionObject),
                                               getInput<moveit_msgs::msg::PlanningScene>(kPortIdPlanningSceneMsg));
  if (!required.has_value())
  {
    return tl::make_unexpected("Failed to get required input: " + required.error());
  }
  const auto& [collision_object_in, planning_scene] = required.value();

  const auto r = getInput<double>(kPortIdR).value_or(kDefaultR);
  const auto g = getInput<double>(kPortIdG).value_or(kDefaultG);
  const auto b = getInput<double>(kPortIdB).value_or(kDefaultB);
  const auto a = getInput<double>(kPortIdA).value_or(kDefaultA);

  auto request = buildRequest(collision_object_in, planning_scene, r, g, b, a);
  if (!request.has_value())
  {
    return tl::make_unexpected(request.error());
  }

  // Capture the prefixed id so processResponse() can name the failing object if the
  // service rejects the request.
  last_request_id_ = request.value().scene.world.collision_objects.back().id;

  spdlog::info("AddColoredVirtualObjectToPlanningScene: adding '{}' with rgba ({:.3f}, {:.3f}, {:.3f}, {:.3f}).",
               last_request_id_, r, g, b, a);

  return request;
}

tl::expected<ApplyPlanningScene::Request, std::string>
AddColoredVirtualObjectToPlanningScene::buildRequest(moveit_msgs::msg::CollisionObject collision_object,
                                                     moveit_msgs::msg::PlanningScene planning_scene, double r, double g,
                                                     double b, double a)
{
  // Fail loudly on out-of-range RGBA rather than silently clamping — the planning-scene
  // visualizer's behavior is undefined for out-of-range components and a silent clamp would
  // hide upstream bugs (e.g. a port mistakenly bound to a 0-255 channel).
  const auto in_unit_range = [](double v) { return v >= 0.0 && v <= 1.0; };
  if (!(in_unit_range(r) && in_unit_range(g) && in_unit_range(b) && in_unit_range(a)))
  {
    return tl::make_unexpected(
        fmt::format("RGBA channel(s) out of range [0, 1]: r={:.3f} g={:.3f} b={:.3f} a={:.3f}.", r, g, b, a));
  }

  // Mirror AddVirtualObjectToPlanningScene's "virtual_" prefix convention so virtual
  // obstacles added through either behavior coexist correctly in the ACM. Use a starts-with
  // test rather than substring search so ids like "my_virtual_box" are correctly classified
  // as not-yet-prefixed.
  collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;
  if (!hasVirtualPrefix(collision_object.id))
  {
    collision_object.id = std::string{ kVirtualPrefix } + collision_object.id;
  }

  moveit_msgs::msg::ObjectColor object_color;
  object_color.id = collision_object.id;
  object_color.color.r = static_cast<float>(r);
  object_color.color.g = static_cast<float>(g);
  object_color.color.b = static_cast<float>(b);
  object_color.color.a = static_cast<float>(a);

  ApplyPlanningScene::Request request;
  request.scene = std::move(planning_scene);
  request.scene.is_diff = true;
  request.scene.world.collision_objects.push_back(collision_object);
  request.scene.object_colors.push_back(object_color);

  // Diagnose accidental double-adds: if the snapshot's ACM already lists this id, the
  // resulting matrix will have duplicate rows/cols and downstream MoveIt lookups behave
  // unpredictably. We don't no-op — that's a behavior change — but we make it loud.
  const auto& existing_entries = request.scene.allowed_collision_matrix.entry_names;
  if (std::find(existing_entries.begin(), existing_entries.end(), collision_object.id) != existing_entries.end())
  {
    spdlog::warn(
        "AddColoredVirtualObjectToPlanningScene: '{}' already exists in the snapshot ACM; the resulting matrix "
        "will contain duplicate entries and downstream collision queries may be unreliable.",
        collision_object.id);
  }

  // Allowed-collision-matrix setup, verbatim from AddVirtualObjectToPlanningScene: forbid
  // collisions with non-virtual objects, allow collisions with other virtual objects
  // (including self). Assumes a square snapshot ACM (entry_values rows match entry_names
  // in length); same precondition as upstream.
  request.scene.allowed_collision_matrix.default_entry_names.push_back(collision_object.id);
  request.scene.allowed_collision_matrix.default_entry_values.push_back(false);

  request.scene.allowed_collision_matrix.entry_names.push_back(collision_object.id);
  moveit_msgs::msg::AllowedCollisionEntry new_entry;
  new_entry.enabled = std::vector<bool>(request.scene.allowed_collision_matrix.entry_names.size(), false);
  new_entry.enabled.back() = true;  // Self-collision allowed.
  request.scene.allowed_collision_matrix.entry_values.push_back(new_entry);

  for (size_t idx = 0; idx < (request.scene.allowed_collision_matrix.entry_names.size() - 1); ++idx)
  {
    const auto& other_id = request.scene.allowed_collision_matrix.entry_names.at(idx);
    if (!hasVirtualPrefix(other_id))
    {
      request.scene.allowed_collision_matrix.entry_values.at(idx).enabled.push_back(false);
    }
    else
    {
      request.scene.allowed_collision_matrix.entry_values.at(idx).enabled.push_back(true);
      request.scene.allowed_collision_matrix.entry_values.back().enabled.at(idx) = true;
    }
  }

  return request;
}

tl::expected<bool, std::string>
AddColoredVirtualObjectToPlanningScene::processResponse(const ApplyPlanningScene::Response& response)
{
  if (!response.success)
  {
    return tl::make_unexpected(
        fmt::format("ApplyPlanningScene service returned success=false for '{}'.", last_request_id_));
  }
  return true;
}

}  // namespace planning_scene_behaviors
