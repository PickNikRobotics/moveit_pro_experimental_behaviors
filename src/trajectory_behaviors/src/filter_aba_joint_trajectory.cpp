#include "trajectory_behaviors/filter_aba_joint_trajectory.hpp"

#include <utility>
#include <vector>

#include <spdlog/spdlog.h>
#include <Eigen/Core>

#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

namespace
{
inline constexpr auto kDescription = R"(
<p>
Takes a JointTrajectory, detects ABA micro-reversals in joint space (A ~= C with direction flip),
removes the middle 'B' points, and optionally removes near-duplicates.
</p>
)";

constexpr auto kPortInJointTrajectory = "joint_trajectory";

constexpr auto kPortInAEqualsCEps = "a_equals_c_eps";
constexpr auto kPortInMinStepEps = "min_step_eps";
constexpr auto kPortInCosThresh = "cos_thresh";
constexpr auto kPortInDownsampleEps = "downsample_eps";

constexpr auto kPortOutJointTrajectory = "corrected_joint_trajectory";
constexpr auto kPortOutABACount = "aba_count";

struct WaypointInfo
{
  Eigen::VectorXd q;
  size_t index;
};

std::vector<WaypointInfo> toWaypoints(const trajectory_msgs::msg::JointTrajectory& jt)
{
  std::vector<WaypointInfo> w;
  w.reserve(jt.points.size());
  for (size_t idx = 0; idx < jt.points.size(); ++idx)
  {
    const auto& pt = jt.points[idx];
    Eigen::VectorXd q(static_cast<Eigen::Index>(pt.positions.size()));
    for (size_t i = 0; i < pt.positions.size(); ++i)
    {
      q(static_cast<Eigen::Index>(i)) = pt.positions[i];
    }
    w.push_back({ q, idx });
  }
  return w;
}

// Remove consecutive near-duplicates by joint-space norm.
// Keeps first+last.
std::vector<WaypointInfo> filterWaypointsByJointDistance(const std::vector<WaypointInfo>& in, double eps)
{
  if (in.empty())
  {
    return {};
  }

  std::vector<WaypointInfo> out;
  out.reserve(in.size());
  out.push_back(in.front());

  for (size_t i = 1; i + 1 < in.size(); ++i)
  {
    if ((in[i].q - out.back().q).norm() >= eps)
    {
      out.push_back(in[i]);
    }
  }

  if (in.size() > 1)
  {
    out.push_back(in.back());
  }

  return out;
}

// Remove ABA micro-reversals: A (kept), B (candidate), C (lookahead).
// If A ~= C and direction flips (cos < cos_thresh), drop B.
std::pair<std::vector<WaypointInfo>, size_t> filterABAMicroReversals(const std::vector<WaypointInfo>& in,
                                                                     double a_equals_c_eps, double min_step_eps,
                                                                     double cos_thresh)
{
  if (in.size() < 3)
  {
    return { in, 0 };
  }

  std::vector<WaypointInfo> out;
  out.reserve(in.size());
  out.push_back(in.front());

  size_t removed = 0;

  for (size_t i = 1; i + 1 < in.size(); ++i)
  {
    const auto& A = out.back().q;
    const auto& B = in[i].q;
    const auto& C = in[i + 1].q;

    const Eigen::VectorXd v1 = B - A;
    const Eigen::VectorXd v2 = C - B;
    const double n1 = v1.norm();
    const double n2 = v2.norm();

    const bool a_equals_c = (C - A).norm() < a_equals_c_eps;

    bool flip = false;
    if (n1 > min_step_eps && n2 > min_step_eps)
    {
      const double cs = v1.dot(v2) / (n1 * n2);
      flip = (cs < cos_thresh);
    }

    if (a_equals_c && flip)
    {
      ++removed;
      continue;
    }

    out.push_back(in[i]);
  }

  out.push_back(in.back());
  return { out, removed };
}

}  // namespace

namespace trajectory_behaviors
{

FilterABAJointTrajectory::FilterABAJointTrajectory(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList FilterABAJointTrajectory::providedPorts()
{
  return {
    BT::InputPort<trajectory_msgs::msg::JointTrajectory>(kPortInJointTrajectory, "{joint_trajectory_msg}",
                                                         "Input JointTrajectory to be checked and corrected."),
    BT::InputPort<double>(kPortInAEqualsCEps, 5e-4, "ABA detection: if ||A - C|| < this, treat as A ~= C."),
    BT::InputPort<double>(kPortInMinStepEps, 1e-12,
                          "ABA detection: ignore steps smaller than this when computing cosine."),
    BT::InputPort<double>(kPortInCosThresh, -0.99, "ABA detection: if cos(v1,v2) < this, treat as direction flip."),
    BT::InputPort<double>(kPortInDownsampleEps, 0.0,
                          "Optional: remove points closer than this (joint-space norm). 0 disables."),

    BT::OutputPort<trajectory_msgs::msg::JointTrajectory>(kPortOutJointTrajectory, "{corrected_joint_trajectory}",
                                                          "Corrected JointTrajectory without retiming."),
    BT::OutputPort<int>(kPortOutABACount, "{aba_count}", "Number of ABA middle points removed."),
  };
}

BT::KeyValueVector FilterABAJointTrajectory::metadata()
{
  return {
    { moveit_pro::behaviors::kSubcategoryMetadataKey, "Utility" },
    { moveit_pro::behaviors::kDescriptionMetadataKey, kDescription },
  };
}

BT::NodeStatus FilterABAJointTrajectory::tick()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(
      getInput<trajectory_msgs::msg::JointTrajectory>(kPortInJointTrajectory), getInput<double>(kPortInAEqualsCEps),
      getInput<double>(kPortInMinStepEps), getInput<double>(kPortInCosThresh), getInput<double>(kPortInDownsampleEps));

  if (!ports)
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "Failed to get required input ports: " + ports.error());
    return BT::NodeStatus::FAILURE;
  }

  const auto& [jt_in, a_equals_c_eps, min_step_eps, cos_thresh, downsample_eps] = ports.value();

  if (jt_in.joint_names.empty() || jt_in.points.size() < 2)
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "Input JointTrajectory has insufficient points.");
    return BT::NodeStatus::FAILURE;
  }

  auto waypoints = toWaypoints(jt_in);
  spdlog::info("ABA correction: input points={} (waypoints={})", jt_in.points.size(), waypoints.size());

  if (downsample_eps > 0.0)
  {
    waypoints = filterWaypointsByJointDistance(waypoints, downsample_eps);
  }

  const auto [filtered, aba_removed] = filterABAMicroReversals(waypoints, a_equals_c_eps, min_step_eps, cos_thresh);
  waypoints = filtered;
  spdlog::info("ABA correction: filtered waypoints={} (aba_removed={})", waypoints.size(), aba_removed);

  if (waypoints.size() < 2)
  {
    getBehaviorContext()->logger->publishWarnMessage(name(), "Filtering left <2 waypoints; passing through input.");
    setOutput(kPortOutJointTrajectory, jt_in);
    setOutput(kPortOutABACount, static_cast<int>(aba_removed));
    return BT::NodeStatus::SUCCESS;
  }

  trajectory_msgs::msg::JointTrajectory jt_out = jt_in;
  jt_out.points.clear();
  jt_out.points.reserve(waypoints.size());
  for (const auto& waypoint : waypoints)
  {
    jt_out.points.push_back(jt_in.points[waypoint.index]);
  }

  setOutput(kPortOutJointTrajectory, jt_out);
  setOutput(kPortOutABACount, static_cast<int>(aba_removed));

  spdlog::info("ABA correction: output points={}", jt_out.points.size());

  return BT::NodeStatus::SUCCESS;
}

}  // namespace trajectory_behaviors
