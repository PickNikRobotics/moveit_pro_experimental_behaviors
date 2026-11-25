// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "geometry_behaviors/axis_rotated_candidate_generation.hpp"

#include <cmath>

namespace geometry_behaviors
{

std::vector<geometry_msgs::msg::PoseStamped> rotateAroundBodyAxis(const geometry_msgs::msg::PoseStamped& input_pose,
                                                                  const Eigen::Vector3d& axis,
                                                                  const std::vector<double>& angles_deg)
{
  const Eigen::Quaterniond q_input(input_pose.pose.orientation.w, input_pose.pose.orientation.x,
                                   input_pose.pose.orientation.y, input_pose.pose.orientation.z);
  const Eigen::Quaterniond q_input_n = q_input.normalized();

  std::vector<geometry_msgs::msg::PoseStamped> candidates;
  candidates.reserve(angles_deg.size());

  for (const double angle_deg : angles_deg)
  {
    const double angle_rad = angle_deg * M_PI / 180.0;
    const Eigen::Quaterniond q_delta(Eigen::AngleAxisd(angle_rad, axis));
    const Eigen::Quaterniond q_out = (q_input_n * q_delta).normalized();

    geometry_msgs::msg::PoseStamped candidate = input_pose;
    candidate.pose.orientation.x = q_out.x();
    candidate.pose.orientation.y = q_out.y();
    candidate.pose.orientation.z = q_out.z();
    candidate.pose.orientation.w = q_out.w();
    candidates.push_back(std::move(candidate));
  }

  return candidates;
}

}  // namespace geometry_behaviors
