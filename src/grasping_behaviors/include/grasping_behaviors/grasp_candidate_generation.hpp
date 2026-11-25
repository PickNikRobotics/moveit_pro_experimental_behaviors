// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_studio_vision_msgs/msg/graspable_object.hpp>
#include <tl_expected/expected.hpp>

#include <Eigen/Geometry>

#include <string>
#include <vector>

namespace grasping_behaviors
{

/**
 * @brief Build a right-handed orientation matrix from an outward-pointing Z-axis direction.
 *
 * Picks a reference axis whose dot with z_axis is small, takes the cross to get X, then crosses
 * Z and X to get Y. The result is right-handed with the supplied direction as Z.
 */
[[nodiscard]] Eigen::Matrix3d buildGraspFrame(const Eigen::Vector3d& z_axis);

/**
 * @brief Generate grasp candidates evenly spaced around the circumference of a cylinder.
 *
 * Emits two end-face candidates first (whichever flat end has the more upward outward normal in
 * world Z; one with Z along the outward end normal, one flipped 180° about local X to point inward),
 * then num_candidates evenly-spaced radial candidates around the circumference at mid-height.
 */
void generateCylinderCandidates(const moveit_studio_vision_msgs::msg::GraspableObject& object, int num_candidates,
                                std::vector<geometry_msgs::msg::PoseStamped>& candidates);

/**
 * @brief Generate grasp candidates centered on the most accessible faces of a box.
 *
 * For each of the 3 axis pairs (+X/-X, +Y/-Y, +Z/-Z), the face whose outward normal has a more
 * positive world-frame Z component is selected. Yields up to 3 candidates and naturally excludes
 * bottom faces.
 */
void generateBoxCandidates(const moveit_studio_vision_msgs::msg::GraspableObject& object,
                           std::vector<geometry_msgs::msg::PoseStamped>& candidates);

/**
 * @brief Generate grasp candidates on a sphere: one at the top, plus a ring tilted from vertical.
 *
 * All grasp poses have their Z-axis pointing radially outward from the sphere center.
 */
void generateSphereCandidates(const moveit_studio_vision_msgs::msg::GraspableObject& object, int num_ring_candidates,
                              double tilt_angle_deg, std::vector<geometry_msgs::msg::PoseStamped>& candidates);

/**
 * @brief Top-level dispatch that branches on the object's BoundingVolume type and produces a
 *        flat list of grasp candidate poses on its surface.
 *
 * Counts: CYLINDER gives 2 end-face + num_cylinder_candidates radial; BOX up to one per axis pair (the
 * more upward-facing face); SPHERE 1 top + num_sphere_ring_candidates ring. Other types return an error.
 *
 * Returns an error if num_cylinder_candidates or num_sphere_ring_candidates is < 1.
 */
[[nodiscard]] tl::expected<std::vector<geometry_msgs::msg::PoseStamped>, std::string>
generateGraspCandidatesForPrimitive(const moveit_studio_vision_msgs::msg::GraspableObject& object,
                                    int num_cylinder_candidates, int num_sphere_ring_candidates,
                                    double sphere_tilt_angle_degrees);

}  // namespace grasping_behaviors
