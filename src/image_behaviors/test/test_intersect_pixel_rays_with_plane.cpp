// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <gtest/gtest.h>

#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include "image_behaviors/register_image_behaviors.hpp"
#include "pixel_ray.hpp"

namespace
{
constexpr auto kCameraFrame = "camera_optical_frame";

sensor_msgs::msg::CameraInfo makeCameraInfo()
{
  sensor_msgs::msg::CameraInfo info;
  info.header.frame_id = kCameraFrame;
  info.width = 640;
  info.height = 480;
  info.k = { 500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0 };
  return info;
}

geometry_msgs::msg::PointStamped pixel(double u_norm, double v_norm)
{
  geometry_msgs::msg::PointStamped point;
  point.point.x = u_norm;
  point.point.y = v_norm;
  return point;
}
}  // namespace

TEST(PixelRay, IntersectsATiltedPlane)
{
  using image_behaviors::pixel_ray::intersectRayWithPlane;
  // Plane through (0, 0, 2) whose normal is tilted away from the optical axis: not an upright plane.
  const Eigen::Vector3d normal = Eigen::Vector3d(0.0, 1.0, -1.0).normalized();
  const auto hit = intersectRayWithPlane(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ(), { 0.0, 0.0, 2.0 }, normal);
  ASSERT_TRUE(hit.has_value());
  EXPECT_NEAR(hit->z(), 2.0, 1e-12);

  // Parallel ray and plane behind the origin both miss.
  EXPECT_FALSE(intersectRayWithPlane(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX(), { 0.0, 0.0, 2.0 },
                                     Eigen::Vector3d::UnitZ())
                   .has_value());
  EXPECT_FALSE(intersectRayWithPlane(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ(), { 0.0, 0.0, -2.0 },
                                     Eigen::Vector3d::UnitZ())
                   .has_value());
}

TEST(PixelRay, CenterPixelLooksAlongTheOpticalAxis)
{
  using namespace image_behaviors::pixel_ray;
  const auto direction = rayDirectionFromNormalizedPixel(0.5, 0.5, intrinsicsFromCameraInfo(makeCameraInfo()));
  EXPECT_NEAR(direction.x(), 0.0, 1e-12);
  EXPECT_NEAR(direction.y(), 0.0, 1e-12);
  EXPECT_NEAR(direction.z(), 1.0, 1e-12);
}

TEST(IntersectPixelRaysWithPlane, ProjectsPixelsAndTheReferencePixel)
{
  auto node = std::make_shared<rclcpp::Node>("test_intersect_pixel_rays_with_plane");
  auto shared_resources = std::make_shared<moveit_pro::behaviors::BehaviorContext>(node, false);

  // A plane facing the camera at a distance of 2 along the optical axis, in the camera frame.
  geometry_msgs::msg::PoseStamped plane;
  plane.header.frame_id = kCameraFrame;
  plane.pose.position.z = 2.0;
  plane.pose.orientation.w = 1.0;

  auto blackboard = BT::Blackboard::create();
  blackboard->set("camera_info", makeCameraInfo());
  blackboard->set("pixels", std::vector<geometry_msgs::msg::PointStamped>{ pixel(0.5, 0.5), pixel(1.0, 0.5) });
  blackboard->set("plane", plane);

  BT::NodeConfiguration config;
  config.blackboard = blackboard;
  config.input_ports["camera_info"] = "{camera_info}";
  config.input_ports["pixel_coords"] = "{pixels}";
  config.input_ports["plane_pose"] = "{plane}";
  config.input_ports["reference_pixel"] = "0.5;1.0";
  config.output_ports["output_poses"] = "{poses}";
  config.output_ports["hit_count"] = "{hit_count}";
  config.output_ports["reference_pose"] = "{reference_pose}";

  // Through the registration function, so the blackboard entries get the port types.
  BT::BehaviorTreeFactory factory;
  image_behaviors::registerImageBehaviors(factory, shared_resources);
  auto behavior = factory.instantiateTreeNode("test", "IntersectPixelRaysWithPlane", config);
  ASSERT_EQ(behavior->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(blackboard->get<int>("hit_count"), 2);

  // The MoveIt Pro BT.CPP stores a vector output as a vector of BT::Any.
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  for (const auto& any : blackboard->getAnyLocked("poses").get()->cast<std::vector<BT::Any>>())
  {
    poses.push_back(any.cast<geometry_msgs::msg::PoseStamped>());
  }
  ASSERT_EQ(poses.size(), 2u);
  EXPECT_NEAR(poses[0].pose.position.x, 0.0, 1e-9);
  EXPECT_NEAR(poses[0].pose.position.z, 2.0, 1e-9);
  // Right image edge: u - cx = width - cx pixels, scaled by depth over fx.
  EXPECT_NEAR(poses[1].pose.position.x, 2.0 * (640.0 - 320.0) / 500.0, 1e-9);
  EXPECT_NEAR(poses[1].pose.position.z, 2.0, 1e-9);

  const auto reference = blackboard->get<geometry_msgs::msg::PoseStamped>("reference_pose");
  EXPECT_NEAR(reference.pose.position.y, 2.0 * (480.0 - 240.0) / 500.0, 1e-9);
  EXPECT_EQ(reference.header.frame_id, kCameraFrame);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
