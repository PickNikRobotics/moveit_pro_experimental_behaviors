// BSD-3-Clause

#include <experimental_behaviors/generate_yaw_poses.hpp>

#include <moveit/task_constructor/storage.h>  // SubTrajectory, InterfaceState
#include <moveit/task_constructor/cost_terms.h>
#include <moveit/planning_scene/planning_scene.hpp>

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Geometry>
#include <cmath>

namespace moveit { namespace task_constructor { namespace stages {

static const rclcpp::Logger LOGGER = rclcpp::get_logger("GenerateYawPoses");

GenerateYawPoses::GenerateYawPoses(const std::string& name) : MonitoringGenerator(name)
{
  setCostTerm(std::make_unique<cost::Constant>(0.0));

  auto& p = properties();
  p.declare<geometry_msgs::msg::PoseStamped>("pose", "base pose to spawn multiple target poses from");
  p.declare<unsigned int>("num_poses", 8, "number of yaw samples to generate");
  p.declare<double>("start_yaw", 0.0, "start yaw offset [rad] relative to base pose");
  p.declare<double>("sweep", 6.283185307179586, "total sweep [rad] about +Z");
}

void GenerateYawPoses::reset()
{
  upstream_solutions_.clear();
  MonitoringGenerator::reset();
}

void GenerateYawPoses::onNewSolution(const SolutionBase& s)
{
  upstream_solutions_.push(&s);
}

bool GenerateYawPoses::canCompute() const
{
  return !upstream_solutions_.empty();
}

void GenerateYawPoses::compute()
{
  if (upstream_solutions_.empty())
    return;

  const SolutionBase& s = *upstream_solutions_.pop();
  planning_scene::PlanningSceneConstPtr scene = s.end()->scene()->diff();

  auto base_pose = properties().get<geometry_msgs::msg::PoseStamped>("pose");
  const auto num = properties().get<unsigned int>("num_poses");
  const double start_yaw = properties().get<double>("start_yaw");
  const double sweep = properties().get<double>("sweep");

  if (num == 0) {
    RCLCPP_WARN(LOGGER, "num_poses must be > 0");
    return;
  }

  if (base_pose.header.frame_id.empty())
    base_pose.header.frame_id = scene->getPlanningFrame();
  else if (!scene->knowsFrameTransform(base_pose.header.frame_id)) {
    RCLCPP_WARN(LOGGER, "Unknown frame: '%s'", base_pose.header.frame_id.c_str());
    return;
  }

  // Base orientation as quaternion
  Eigen::Quaterniond q_base(base_pose.pose.orientation.w,
                            base_pose.pose.orientation.x,
                            base_pose.pose.orientation.y,
                            base_pose.pose.orientation.z);
  if (q_base.norm() <= std::numeric_limits<double>::epsilon())
    q_base = Eigen::Quaterniond::Identity();
  else
    q_base.normalize();

  const double step = (num == 1) ? 0.0 : (sweep / static_cast<double>(num));
  for (unsigned int i = 0; i < num; ++i) {
    const double yaw = start_yaw + static_cast<double>(i) * step;
    const Eigen::AngleAxisd aa_yaw(yaw, Eigen::Vector3d::UnitZ());
    const Eigen::Quaterniond q_out = aa_yaw * q_base;

    geometry_msgs::msg::PoseStamped pose_i = base_pose;
    pose_i.pose.orientation.w = q_out.w();
    pose_i.pose.orientation.x = q_out.x();
    pose_i.pose.orientation.y = q_out.y();
    pose_i.pose.orientation.z = q_out.z();

    InterfaceState state(scene);
    forwardProperties(*s.end(), state);
    state.properties().set("target_pose", pose_i);

    SubTrajectory trajectory;
    trajectory.setCost(0.0);
    spawn(std::move(state), std::move(trajectory));
  }
}

}}}  // namespace moveit::task_constructor::stages
