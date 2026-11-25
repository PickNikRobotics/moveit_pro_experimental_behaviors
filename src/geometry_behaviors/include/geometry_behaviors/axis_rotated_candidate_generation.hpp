// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <Eigen/Geometry>

#include <vector>

namespace geometry_behaviors
{

/**
 * @brief Produce a list of pose candidates by rotating an input pose about its body-frame axis
 *        by each of the supplied angles in turn.
 *
 * The position and frame_id of the input pose are preserved on every output candidate. The output
 * orientation is q_input * AngleAxisd(angle_rad, axis), so the rotation is applied in the input
 * pose's body frame (right-multiply). Output candidates appear in input-angle order; an empty
 * angles list produces an empty output vector.
 *
 * @param input_pose  Base pose. Position and frame_id are reused on every candidate.
 * @param axis        Body-frame rotation axis (unit vector; not normalized internally).
 * @param angles_deg  Rotation angles in degrees, applied one per output candidate.
 */
[[nodiscard]] std::vector<geometry_msgs::msg::PoseStamped>
rotateAroundBodyAxis(const geometry_msgs::msg::PoseStamped& input_pose, const Eigen::Vector3d& axis,
                     const std::vector<double>& angles_deg);

}  // namespace geometry_behaviors
