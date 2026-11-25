// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <gtest/gtest.h>

#include <array>
#include <cstring>
#include <limits>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "point_cloud_behaviors/crop_point_cloud_to_camera_frustum.hpp"

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

// An unordered XYZ cloud with one extra FLOAT32 field, to check that other fields are kept.
sensor_msgs::msg::PointCloud2 makeCloud(const std::vector<std::array<float, 4>>& points)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = kCameraFrame;
  const char* names[] = { "x", "y", "z", "intensity" };
  for (int i = 0; i < 4; ++i)
  {
    sensor_msgs::msg::PointField field;
    field.name = names[i];
    field.offset = static_cast<std::uint32_t>(i * sizeof(float));
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1;
    cloud.fields.push_back(field);
  }
  cloud.point_step = 4 * sizeof(float);
  cloud.height = 1;
  cloud.width = static_cast<std::uint32_t>(points.size());
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.data.resize(cloud.row_step);
  std::memcpy(cloud.data.data(), points.data(), cloud.data.size());
  return cloud;
}

class CropPointCloudToCameraFrustumTest : public testing::Test
{
protected:
  BT::NodeStatus run(const sensor_msgs::msg::PointCloud2& cloud, double min_range, double max_range)
  {
    blackboard_ = BT::Blackboard::create();
    blackboard_->set("point_cloud", cloud);
    blackboard_->set("camera_info", makeCameraInfo());
    BT::NodeConfiguration config;
    config.blackboard = blackboard_;
    config.input_ports["point_cloud"] = "{point_cloud}";
    config.input_ports["camera_info"] = "{camera_info}";
    config.input_ports["min_range"] = std::to_string(min_range);
    config.input_ports["max_range"] = std::to_string(max_range);
    config.input_ports["pixel_margin"] = "0";
    config.output_ports["point_cloud_cropped"] = "{cropped}";
    config.output_ports["kept_count"] = "{kept_count}";
    point_cloud_behaviors::CropPointCloudToCameraFrustum behavior("test", config, shared_resources_);
    return behavior.executeTick();
  }

  std::shared_ptr<rclcpp::Node> node_ = std::make_shared<rclcpp::Node>("test_crop_point_cloud_to_camera_frustum");
  std::shared_ptr<moveit_pro::behaviors::BehaviorContext> shared_resources_ =
      std::make_shared<moveit_pro::behaviors::BehaviorContext>(node_, false);
  BT::Blackboard::Ptr blackboard_;
};
}  // namespace

TEST_F(CropPointCloudToCameraFrustumTest, KeepsOnlyPointsInsideTheFrustum)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const auto cloud = makeCloud({
      { 0.0F, 0.0F, 1.0F, 7.0F },   // On the optical axis, inside: kept.
      { 0.0F, 0.0F, 0.2F, 1.0F },   // Nearer than min_range.
      { 0.0F, 0.0F, 5.0F, 2.0F },   // Further than max_range.
      { 2.0F, 0.0F, 1.0F, 3.0F },   // Projects to the right of the image.
      { 0.0F, 0.0F, -1.0F, 4.0F },  // Behind the camera.
      { nan, nan, nan, 5.0F },      // Invalid depth.
      { 0.2F, -0.1F, 2.0F, 9.0F },  // Inside, off axis: kept.
  });
  ASSERT_EQ(run(cloud, 0.5, 3.0), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(blackboard_->get<int>("kept_count"), 2);

  const auto cropped = blackboard_->get<sensor_msgs::msg::PointCloud2>("cropped");
  EXPECT_EQ(cropped.width, 2u);
  EXPECT_EQ(cropped.height, 1u);
  EXPECT_EQ(cropped.header.frame_id, kCameraFrame);
  ASSERT_EQ(cropped.data.size(), 2 * cloud.point_step);
  std::array<float, 8> kept{};
  std::memcpy(kept.data(), cropped.data.data(), cropped.data.size());
  EXPECT_FLOAT_EQ(kept[3], 7.0F);
  EXPECT_FLOAT_EQ(kept[7], 9.0F);
}

TEST_F(CropPointCloudToCameraFrustumTest, InvalidRangeFails)
{
  const auto cloud = makeCloud({ { 0.0F, 0.0F, 1.0F, 0.0F } });
  EXPECT_EQ(run(cloud, 2.0, 1.0), BT::NodeStatus::FAILURE);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
