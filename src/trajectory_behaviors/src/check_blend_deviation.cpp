#include "trajectory_behaviors/check_blend_deviation.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include <spdlog/spdlog.h>
#include <Eigen/Core>

#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

namespace
{
inline constexpr auto kDescription = R"(
<p>
Rejects a blended trajectory that strays too far in joint space from the original un-blended path,
checked only within the blended regions.
</p>
<p>
BlendJointTrajectories does no collision checking and applies no path-deviation bound, so a large
blending_radius can produce a smooth Ruckig segment that loops away from the inputs and into collision.
This Behavior checks deviation only inside each blend zone: a zone is the set of points (in both the
blended trajectory and the original concatenated path) within <tt>blend_radius</tt> radians
(joint-space Euclidean distance) of the connection point between two consecutive input trajectories.
<tt>blend_radius</tt> must match the value passed to BlendJointTrajectories.
No collision checking and no forward kinematics.
</p>
)";

constexpr auto kPortInOriginalTrajectories = "original_trajectories";
constexpr auto kPortInBlendedTrajectory = "blended_trajectory";
constexpr auto kPortInMaxDeviation = "max_joint_deviation";
constexpr auto kPortInBlendRadius = "blend_radius";
constexpr auto kPortOutMeasuredDeviation = "measured_deviation";

// Maps `from`'s joint order onto `to`'s joint order: the returned vector `perm` satisfies
// from[perm[i]] == to[i], so a position vector in `from`'s order can be reordered into `to`'s
// order via `reordered[i] = original[perm[i]]`. Returns nullopt if the two don't contain the
// same set of joint names.
std::optional<std::vector<Eigen::Index>> buildPermutation(const std::vector<std::string>& from,
                                                          const std::vector<std::string>& to)
{
  if (from.size() != to.size())
    return std::nullopt;

  std::vector<Eigen::Index> perm(to.size());
  for (size_t i = 0; i < to.size(); ++i)
  {
    const auto it = std::find(from.begin(), from.end(), to[i]);
    if (it == from.end())
      return std::nullopt;
    perm[i] = static_cast<Eigen::Index>(std::distance(from.begin(), it));
  }
  return perm;
}

// Flatten a JointTrajectory's positions into a list of Eigen vectors.
std::vector<Eigen::VectorXd> toPositions(const trajectory_msgs::msg::JointTrajectory& jt)
{
  std::vector<Eigen::VectorXd> out;
  out.reserve(jt.points.size());
  for (const auto& pt : jt.points)
  {
    out.emplace_back(
        Eigen::Map<const Eigen::VectorXd>(pt.positions.data(), static_cast<Eigen::Index>(pt.positions.size())));
  }
  return out;
}

// Same as above, but reordered per `permutation` (i.e. `out[i] = jt.points[*].positions[permutation[i]]`)
// so all trajectories end up in the same joint order regardless of how each trajectory's joint_names
// happened to be listed.
std::vector<Eigen::VectorXd> toPositions(const trajectory_msgs::msg::JointTrajectory& jt,
                                         const std::vector<Eigen::Index>& permutation)
{
  std::vector<Eigen::VectorXd> out;
  out.reserve(jt.points.size());
  for (const auto& pt : jt.points)
  {
    Eigen::VectorXd reordered(static_cast<Eigen::Index>(permutation.size()));
    for (size_t i = 0; i < permutation.size(); ++i)
      reordered[static_cast<Eigen::Index>(i)] = pt.positions[static_cast<size_t>(permutation[i])];
    out.emplace_back(std::move(reordered));
  }
  return out;
}

}  // namespace

namespace trajectory_behaviors
{

CheckBlendDeviation::CheckBlendDeviation(const std::string& name, const BT::NodeConfiguration& config,
                                         const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList CheckBlendDeviation::providedPorts()
{
  return {
    BT::InputPort<std::vector<trajectory_msgs::msg::JointTrajectory>>(kPortInOriginalTrajectories, "{jt_vector}",
                                                                      "The un-blended input trajectories that were fed "
                                                                      "to BlendJointTrajectories."),
    BT::InputPort<trajectory_msgs::msg::JointTrajectory>(kPortInBlendedTrajectory, "{connected_joint_trajectory}",
                                                         "The blended output of BlendJointTrajectories."),
    BT::InputPort<double>(kPortInMaxDeviation, 0.10,
                          "Max allowed joint-space distance (rad) of any blended point from the reference path "
                          "within the blend zone."),
    BT::InputPort<double>(kPortInBlendRadius,
                          "Joint-space Euclidean distance (rad) from each connection point that defines the blend "
                          "zone. Must match the blend_radius passed to BlendJointTrajectories."),
    BT::OutputPort<double>(kPortOutMeasuredDeviation, "{blend_deviation}",
                           "The largest joint-space deviation found within any blend zone (rad)."),
  };
}

BT::KeyValueVector CheckBlendDeviation::metadata()
{
  return {
    { moveit_pro::behaviors::kSubcategoryMetadataKey, "Utility" },
    { moveit_pro::behaviors::kDescriptionMetadataKey, kDescription },
  };
}

BT::NodeStatus CheckBlendDeviation::tick()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(
      getInput<std::vector<trajectory_msgs::msg::JointTrajectory>>(kPortInOriginalTrajectories),
      getInput<trajectory_msgs::msg::JointTrajectory>(kPortInBlendedTrajectory), getInput<double>(kPortInMaxDeviation),
      getInput<double>(kPortInBlendRadius));

  if (!ports)
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "Failed to get required input ports: " + ports.error());
    return BT::NodeStatus::FAILURE;
  }

  const auto& [originals, blended, max_deviation, blend_radius] = ports.value();

  if (originals.empty() || blended.points.empty())
  {
    getBehaviorContext()->logger->publishFailureMessage(name(),
                                                        "original_trajectories or blended_trajectory is empty.");
    return BT::NodeStatus::FAILURE;
  }

  if (originals.size() < 2)
  {
    spdlog::info("CheckBlendDeviation: only one trajectory provided, no blend zones to check.");
    setOutput(kPortOutMeasuredDeviation, 0.0);
    return BT::NodeStatus::SUCCESS;
  }

  // Build the reference path: all points from the original trajectories, concatenated in order.
  // If a trajectory's joint_names already matches the blended trajectory's exactly, flatten it
  // as-is; only remap joints into the blended trajectory's order when the ordering actually
  // differs, since BlendJointTrajectories' inputs aren't guaranteed to agree on joint order.
  std::vector<Eigen::VectorXd> reference;
  for (size_t i = 0; i < originals.size(); ++i)
  {
    if (originals[i].points.empty())
    {
      getBehaviorContext()->logger->publishFailureMessage(name(), "original_trajectories[" + std::to_string(i) +
                                                                      "] has no points.");
      return BT::NodeStatus::FAILURE;
    }

    // Validate point DOF counts before flattening/reordering to avoid out-of-bounds access.
    const size_t expected_dofs = originals[i].joint_names.size();
    for (const auto& pt : originals[i].points)
    {
      if (pt.positions.size() != expected_dofs)
      {
        getBehaviorContext()->logger->publishFailureMessage(
            name(),
            "original_trajectories[" + std::to_string(i) + "] point positions size does not match joint_names size.");
        return BT::NodeStatus::FAILURE;
      }
    }

    std::vector<Eigen::VectorXd> positions;
    if (originals[i].joint_names == blended.joint_names)
    {
      positions = toPositions(originals[i]);
    }
    else
    {
      getBehaviorContext()->logger->publishWarnMessage(
          name(), "original_trajectories[" + std::to_string(i) +
                      "].joint_names does not exactly match blended_trajectory.joint_names. Attempting to re-order.");
      const auto permutation = buildPermutation(originals[i].joint_names, blended.joint_names);
      if (!permutation)
      {
        getBehaviorContext()->logger->publishFailureMessage(
            name(), "original_trajectories[" + std::to_string(i) +
                        "].joint_names does not contain the same joints as blended_trajectory.joint_names.");
        return BT::NodeStatus::FAILURE;
      }
      positions = toPositions(originals[i], *permutation);
    }
    reference.insert(reference.end(), positions.begin(), positions.end());
  }

  const Eigen::Index dofs = reference.front().size();
  const int N_ref = static_cast<int>(reference.size());
  const int N_blended = static_cast<int>(blended.points.size());

  if (N_ref < 2 || N_blended < 2)
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "Trajectories must have at least two points.");
    return BT::NodeStatus::FAILURE;
  }

  // Normalized index t = i/(N-1) stands in for time, so retiming after blending does not matter. Per seam:
  // reference zone within blend_radius, its t range, then each blended point in that range against the
  // reference point at the same t. Steps in docs/AGENT_COMMENTS.md.

  double worst_deviation = 0.0;
  size_t checked_points = 0;

  int ref_offset = 0;
  for (size_t k = 0; k + 1 < originals.size(); ++k)
  {
    const int n_k = static_cast<int>(originals[k].points.size());
    const int n_k1 = static_cast<int>(originals[k + 1].points.size());

    // Index of the seam (last point of trajectory k) in the concatenated reference array.
    const int conn_ref_idx = ref_offset + n_k - 1;
    const Eigen::VectorXd& conn_pt = reference[conn_ref_idx];

    // Scan backward through trajectory k; stop at the start of trajectory k.
    int lo_ref = conn_ref_idx;
    while (lo_ref > ref_offset && (reference[lo_ref - 1] - conn_pt).norm() <= blend_radius)
      --lo_ref;

    // Scan forward through trajectory k+1; stop at the end of trajectory k+1.
    const int traj_k1_end = ref_offset + n_k + n_k1 - 1;
    int hi_ref = conn_ref_idx;
    while (hi_ref < traj_k1_end && (reference[hi_ref + 1] - conn_pt).norm() <= blend_radius)
      ++hi_ref;

    const double t_lo = static_cast<double>(lo_ref) / (N_ref - 1);
    const double t_hi = static_cast<double>(hi_ref) / (N_ref - 1);

    for (int j = 0; j < N_blended; ++j)
    {
      const double t_j = static_cast<double>(j) / (N_blended - 1);
      if (t_j < t_lo || t_j > t_hi)
        continue;

      const auto& pt = blended.points[j];
      if (static_cast<Eigen::Index>(pt.positions.size()) != dofs)
      {
        getBehaviorContext()->logger->publishFailureMessage(
            name(), "Blended trajectory DOF count does not match the original trajectories.");
        return BT::NodeStatus::FAILURE;
      }
      const Eigen::Map<const Eigen::VectorXd> p(pt.positions.data(), dofs);

      // Time-corresponding reference point: round t_j back to the nearest reference index.
      const int ref_idx = static_cast<int>(std::round(t_j * (N_ref - 1)));
      worst_deviation = std::max(worst_deviation, (p - reference[ref_idx]).norm());
      ++checked_points;
    }

    ref_offset += n_k;
  }

  setOutput(kPortOutMeasuredDeviation, worst_deviation);

  if (checked_points == 0)
  {
    spdlog::warn("CheckBlendDeviation: no blended points fell within any blend zone "
                 "(blend_radius={:.4f} rad). Deviation check was vacuously satisfied.",
                 blend_radius);
    return BT::NodeStatus::SUCCESS;
  }

  spdlog::info("CheckBlendDeviation: checked {} point(s) across {} blend zone(s); max deviation {:.4f} rad.",
               checked_points, originals.size() - 1, worst_deviation);

  if (worst_deviation > max_deviation)
  {
    getBehaviorContext()->logger->publishWarnMessage(name(), "Blend deviates " + std::to_string(worst_deviation) +
                                                                 " rad from the original path (limit " +
                                                                 std::to_string(max_deviation) + " rad); rejecting.");
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace trajectory_behaviors
