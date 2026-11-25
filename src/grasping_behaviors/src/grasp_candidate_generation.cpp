// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "grasping_behaviors/grasp_candidate_generation.hpp"

#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <array>
#include <cmath>
#include <string>

namespace grasping_behaviors
{

Eigen::Matrix3d buildGraspFrame(const Eigen::Vector3d& z_axis)
{
  const Eigen::Vector3d ref =
      (std::abs(z_axis.dot(Eigen::Vector3d::UnitZ())) < 0.9) ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitX();
  const Eigen::Vector3d x_axis = ref.cross(z_axis).normalized();
  const Eigen::Vector3d y_axis = z_axis.cross(x_axis);

  Eigen::Matrix3d rotation;
  rotation.col(0) = x_axis;
  rotation.col(1) = y_axis;
  rotation.col(2) = z_axis;
  return rotation;
}

void generateCylinderCandidates(const moveit_studio_vision_msgs::msg::GraspableObject& object, int num_candidates,
                                std::vector<geometry_msgs::msg::PoseStamped>& candidates)
{
  const double radius = object.grasp_info.bounding_volume.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS];
  const double height = object.grasp_info.bounding_volume.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT];

  Eigen::Isometry3d cylinder_pose;
  tf2::fromMsg(object.object.pose, cylinder_pose);

  // End-face candidates on whichever flat end points more upward, prepended so MTC tries a top-down grip
  // first: one outward and one inward (flipped about local X) orientation. See docs/AGENT_COMMENTS.md.
  {
    const double world_z_of_plus_end = (cylinder_pose.linear() * Eigen::Vector3d::UnitZ()).z();
    const bool top_is_plus_end = world_z_of_plus_end >= 0.0;
    const double end_z_local = top_is_plus_end ? (height / 2.0) : -(height / 2.0);
    const Eigen::Vector3d end_normal_local(0.0, 0.0, top_is_plus_end ? 1.0 : -1.0);

    Eigen::Isometry3d local_end_grasp_outward = Eigen::Isometry3d::Identity();
    local_end_grasp_outward.translation() = Eigen::Vector3d(0.0, 0.0, end_z_local);

    // Right-handed frame: Z along outward end normal, X along cylinder's local +X, Y chosen to complete.
    const Eigen::Vector3d end_x = Eigen::Vector3d::UnitX();
    const Eigen::Vector3d end_y = end_normal_local.cross(end_x);
    Eigen::Matrix3d end_rotation;
    end_rotation.col(0) = end_x;
    end_rotation.col(1) = end_y;
    end_rotation.col(2) = end_normal_local;
    local_end_grasp_outward.linear() = end_rotation;

    geometry_msgs::msg::PoseStamped ps_end_outward;
    ps_end_outward.header = object.object.header;
    ps_end_outward.pose = tf2::toMsg(cylinder_pose * local_end_grasp_outward);
    candidates.push_back(ps_end_outward);

    const Eigen::Isometry3d local_end_grasp_inward =
        local_end_grasp_outward * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
    geometry_msgs::msg::PoseStamped ps_end_inward;
    ps_end_inward.header = object.object.header;
    ps_end_inward.pose = tf2::toMsg(cylinder_pose * local_end_grasp_inward);
    candidates.push_back(ps_end_inward);
  }

  const double angle_step = 2.0 * M_PI / num_candidates;

  for (int i = 0; i < num_candidates; ++i)
  {
    const double theta = angle_step * i;
    const double ct = std::cos(theta);
    const double st = std::sin(theta);

    // Position on the circumference at mid-height (z=0 in the cylinder's local frame).
    Eigen::Isometry3d local_grasp = Eigen::Isometry3d::Identity();
    local_grasp.translation() = Eigen::Vector3d(radius * ct, radius * st, 0.0);

    // Z-axis radially outward, Y-axis along cylinder axis, X-axis tangent.
    Eigen::Matrix3d rotation;
    rotation.col(0) = Eigen::Vector3d(-st, ct, 0.0);
    rotation.col(1) = Eigen::Vector3d(0.0, 0.0, 1.0);
    rotation.col(2) = Eigen::Vector3d(ct, st, 0.0);
    local_grasp.linear() = rotation;

    geometry_msgs::msg::PoseStamped ps;
    ps.header = object.object.header;
    ps.pose = tf2::toMsg(cylinder_pose * local_grasp);
    candidates.push_back(ps);
  }
}

void generateBoxCandidates(const moveit_studio_vision_msgs::msg::GraspableObject& object,
                           std::vector<geometry_msgs::msg::PoseStamped>& candidates)
{
  const auto& dims = object.grasp_info.bounding_volume.dimensions;
  const double dx = dims[shape_msgs::msg::SolidPrimitive::BOX_X];
  const double dy = dims[shape_msgs::msg::SolidPrimitive::BOX_Y];
  const double dz = dims[shape_msgs::msg::SolidPrimitive::BOX_Z];

  Eigen::Isometry3d box_pose;
  tf2::fromMsg(object.object.pose, box_pose);

  // Each face is defined by its center, outward normal (z), and tangent axes (x, y)
  // forming a right-handed frame.
  struct FaceInfo
  {
    Eigen::Vector3d center;
    Eigen::Vector3d normal;
    Eigen::Vector3d x_axis;
    Eigen::Vector3d y_axis;
  };

  // clang-format off
  const std::array<FaceInfo, 6> faces = {{
    { { dx / 2, 0, 0}, { 1, 0, 0}, {0,  1, 0}, {0, 0, 1} },  // +X
    { {-dx / 2, 0, 0}, {-1, 0, 0}, {0, -1, 0}, {0, 0, 1} },  // -X
    { {0,  dy / 2, 0}, {0,  1, 0}, {-1, 0, 0}, {0, 0, 1} },  // +Y
    { {0, -dy / 2, 0}, {0, -1, 0}, { 1, 0, 0}, {0, 0, 1} },  // -Y
    { {0, 0,  dz / 2}, {0, 0,  1}, { 1, 0, 0}, {0, 1, 0} },  // +Z
    { {0, 0, -dz / 2}, {0, 0, -1}, {-1, 0, 0}, {0, 1, 0} },  // -Z
  }};
  // clang-format on

  for (int pair = 0; pair < 3; ++pair)
  {
    const auto& face_pos = faces[pair * 2];
    const auto& face_neg = faces[pair * 2 + 1];

    const double z_pos = (box_pose.linear() * face_pos.normal).z();
    const double z_neg = (box_pose.linear() * face_neg.normal).z();
    const auto& face = (z_pos >= z_neg) ? face_pos : face_neg;

    Eigen::Isometry3d local_grasp = Eigen::Isometry3d::Identity();
    local_grasp.translation() = face.center;
    Eigen::Matrix3d rotation;
    rotation.col(0) = face.x_axis;
    rotation.col(1) = face.y_axis;
    rotation.col(2) = face.normal;
    local_grasp.linear() = rotation;

    geometry_msgs::msg::PoseStamped ps;
    ps.header = object.object.header;
    ps.pose = tf2::toMsg(box_pose * local_grasp);
    candidates.push_back(ps);
  }
}

void generateSphereCandidates(const moveit_studio_vision_msgs::msg::GraspableObject& object, int num_ring_candidates,
                              double tilt_angle_deg, std::vector<geometry_msgs::msg::PoseStamped>& candidates)
{
  const double radius = object.grasp_info.bounding_volume.dimensions[shape_msgs::msg::SolidPrimitive::SPHERE_RADIUS];

  const Eigen::Vector3d center(object.object.pose.position.x, object.object.pose.position.y,
                               object.object.pose.position.z);

  // Top grasp: highest point on the sphere in world Z.
  {
    const Eigen::Vector3d z_axis = Eigen::Vector3d::UnitZ();
    Eigen::Isometry3d grasp = Eigen::Isometry3d::Identity();
    grasp.translation() = center + radius * z_axis;
    grasp.linear() = buildGraspFrame(z_axis);

    geometry_msgs::msg::PoseStamped ps;
    ps.header = object.object.header;
    ps.pose = tf2::toMsg(grasp);
    candidates.push_back(ps);
  }

  // Ring of grasps tilted from the top.
  const double alpha = tilt_angle_deg * M_PI / 180.0;
  const double phi_step = 2.0 * M_PI / num_ring_candidates;

  for (int i = 0; i < num_ring_candidates; ++i)
  {
    const double phi = phi_step * i;
    const Eigen::Vector3d direction(std::sin(alpha) * std::cos(phi), std::sin(alpha) * std::sin(phi), std::cos(alpha));

    Eigen::Isometry3d grasp = Eigen::Isometry3d::Identity();
    grasp.translation() = center + radius * direction;
    grasp.linear() = buildGraspFrame(direction);

    geometry_msgs::msg::PoseStamped ps;
    ps.header = object.object.header;
    ps.pose = tf2::toMsg(grasp);
    candidates.push_back(ps);
  }
}

tl::expected<std::vector<geometry_msgs::msg::PoseStamped>, std::string>
generateGraspCandidatesForPrimitive(const moveit_studio_vision_msgs::msg::GraspableObject& object,
                                    int num_cylinder_candidates, int num_sphere_ring_candidates,
                                    double sphere_tilt_angle_degrees)
{
  if (num_cylinder_candidates < 1)
  {
    return tl::make_unexpected("num_cylinder_candidates must be at least 1.");
  }
  if (num_sphere_ring_candidates < 1)
  {
    return tl::make_unexpected("num_sphere_ring_candidates must be at least 1.");
  }

  std::vector<geometry_msgs::msg::PoseStamped> candidates;

  switch (object.grasp_info.bounding_volume.type)
  {
    case shape_msgs::msg::SolidPrimitive::CYLINDER:
      generateCylinderCandidates(object, num_cylinder_candidates, candidates);
      break;
    case shape_msgs::msg::SolidPrimitive::BOX:
      generateBoxCandidates(object, candidates);
      break;
    case shape_msgs::msg::SolidPrimitive::SPHERE:
      generateSphereCandidates(object, num_sphere_ring_candidates, sphere_tilt_angle_degrees, candidates);
      break;
    default:
      return tl::make_unexpected("Unsupported shape primitive type: " +
                                 std::to_string(object.grasp_info.bounding_volume.type));
  }

  return candidates;
}

}  // namespace grasping_behaviors
