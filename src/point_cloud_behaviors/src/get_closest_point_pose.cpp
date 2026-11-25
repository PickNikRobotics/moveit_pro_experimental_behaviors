#include "point_cloud_behaviors/get_closest_point_pose.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Geometry>
#include <cmath>

namespace
{
constexpr auto kPortIDPointCloud = "point_cloud";
constexpr auto kPortIDInputPose = "input_pose";
constexpr auto kPortIDNormalKNeighbors = "normal_k_neighbors";
constexpr auto kPortIDOutputPose = "closest_point_pose";

inline constexpr auto kDescription = R"(
    <p>Finds the closest point in a point cloud to a reference pose and returns a new pose
    at that point with the Z axis aligned along the surface normal.</p>
    <p>The point cloud and input pose must be in the same frame. The surface normal is
    estimated using the local neighborhood around the closest point and oriented to
    point toward the input pose position.</p>
)";
}  // namespace

namespace point_cloud_behaviors
{

GetClosestPointPose::GetClosestPointPose(const std::string& name, const BT::NodeConfiguration& config,
                                         const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : SharedResourcesNode(name, config, shared_resources)
{
}

BT::PortsList GetClosestPointPose::providedPorts()
{
  return {
    BT::InputPort<sensor_msgs::msg::PointCloud2>(kPortIDPointCloud, "Input point cloud."),
    BT::InputPort<geometry_msgs::msg::PoseStamped>(kPortIDInputPose, "Reference pose to find closest point to."),
    BT::InputPort<int>(kPortIDNormalKNeighbors, 30, "Number of neighbors for surface normal estimation."),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>(kPortIDOutputPose, "{closest_point_pose}",
                                                    "Pose at the closest point with Z axis along the surface normal."),
  };
}

BT::KeyValueVector GetClosestPointPose::metadata()
{
  return { { moveit_pro::behaviors::kDescriptionMetadataKey, kDescription },
           { moveit_pro::behaviors::kSubcategoryMetadataKey, "Perception - 3D" } };
}

BT::NodeStatus GetClosestPointPose::tick()
{
  const auto ports =
      moveit_pro::behaviors::getRequiredInputs(getInput<sensor_msgs::msg::PointCloud2>(kPortIDPointCloud),
                                               getInput<geometry_msgs::msg::PoseStamped>(kPortIDInputPose));

  if (!ports.has_value())
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "Failed to get required inputs: " + ports.error());
    return BT::NodeStatus::FAILURE;
  }

  const auto& [point_cloud_msg, input_pose] = ports.value();

  // Verify that the point cloud and pose are in the same frame.
  if (point_cloud_msg.header.frame_id != input_pose.header.frame_id)
  {
    getBehaviorContext()->logger->publishFailureMessage(
        name(), "Frame mismatch: point cloud is in '" + point_cloud_msg.header.frame_id + "' but pose is in '" +
                    input_pose.header.frame_id + "'. Transform them into the same frame before calling this behavior.");
    return BT::NodeStatus::FAILURE;
  }

  const int k_neighbors = getInput<int>(kPortIDNormalKNeighbors).value_or(30);

  // Convert ROS PointCloud2 to PCL.
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromROSMsg(point_cloud_msg, *cloud);

  if (cloud->empty())
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "Point cloud is empty.");
    return BT::NodeStatus::FAILURE;
  }

  // Find the closest point using a KD-tree.
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(cloud);

  pcl::PointXYZ query;
  query.x = static_cast<float>(input_pose.pose.position.x);
  query.y = static_cast<float>(input_pose.pose.position.y);
  query.z = static_cast<float>(input_pose.pose.position.z);

  std::vector<int> indices(1);
  std::vector<float> distances(1);
  if (kdtree.nearestKSearch(query, 1, indices, distances) < 1)
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "Nearest-neighbor search failed.");
    return BT::NodeStatus::FAILURE;
  }

  const auto& closest_point = cloud->points[indices[0]];

  // Estimate the surface normal at the closest point.
  // Setting the viewpoint to the input pose position ensures the normal
  // points toward the pose (i.e. outward from the surface toward the robot).
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(cloud);
  ne.setSearchMethod(std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>());
  ne.setKSearch(k_neighbors);
  ne.setViewPoint(query.x, query.y, query.z);

  auto point_indices = std::make_shared<pcl::PointIndices>();
  point_indices->indices.push_back(indices[0]);
  ne.setIndices(point_indices);

  pcl::PointCloud<pcl::Normal> normals;
  ne.compute(normals);

  if (normals.empty() || !std::isfinite(normals.points[0].normal_x))
  {
    getBehaviorContext()->logger->publishFailureMessage(name(),
                                                        "Failed to estimate surface normal at the closest point.");
    return BT::NodeStatus::FAILURE;
  }

  Eigen::Vector3d normal(normals.points[0].normal_x, normals.points[0].normal_y, normals.points[0].normal_z);
  normal.normalize();

  // Build an orientation where the Z axis is the surface normal.
  Eigen::Vector3d z_axis = normal;

  // Choose a reference vector that is not parallel to the normal.
  Eigen::Vector3d ref =
      (std::abs(z_axis.dot(Eigen::Vector3d::UnitX())) < 0.9) ? Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();

  Eigen::Vector3d x_axis = ref.cross(z_axis).normalized();
  Eigen::Vector3d y_axis = z_axis.cross(x_axis);

  Eigen::Matrix3d rotation;
  rotation.col(0) = x_axis;
  rotation.col(1) = y_axis;
  rotation.col(2) = z_axis;

  Eigen::Isometry3d output_transform = Eigen::Isometry3d::Identity();
  output_transform.translation() = Eigen::Vector3d(closest_point.x, closest_point.y, closest_point.z);
  output_transform.linear() = rotation;

  // Build output PoseStamped.
  geometry_msgs::msg::PoseStamped output_pose;
  output_pose.header.frame_id = point_cloud_msg.header.frame_id;
  output_pose.header.stamp = getBehaviorContext()->node->now();
  output_pose.pose = tf2::toMsg(output_transform);

  setOutput(kPortIDOutputPose, output_pose);

  getBehaviorContext()->logger->publishInfoMessage(
      name(), "Closest point at (" + std::to_string(closest_point.x) + ", " + std::to_string(closest_point.y) + ", " +
                  std::to_string(closest_point.z) + "), distance=" + std::to_string(std::sqrt(distances[0])) +
                  "m, normal=(" + std::to_string(normal.x()) + ", " + std::to_string(normal.y()) + ", " +
                  std::to_string(normal.z()) + ")");

  return BT::NodeStatus::SUCCESS;
}

}  // namespace point_cloud_behaviors
