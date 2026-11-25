// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>

#include <behaviortree_cpp/bt_factory.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "point_cloud_behaviors/load_latest_point_cloud_from_file.hpp"

using point_cloud_behaviors::internal::findNewestPcdFile;

namespace
{
class LoadLatestPointCloudFromFileTest : public testing::Test
{
protected:
  void SetUp() override
  {
    directory_ = std::filesystem::temp_directory_path() /
                 ("load_latest_pcd_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(directory_);
  }

  void TearDown() override
  {
    std::filesystem::remove_all(directory_);
  }

  // Writes a cloud of @p point_count points and sets its modification time @p age_s seconds in the past.
  std::filesystem::path writeCloud(const std::string& file_name, std::size_t point_count, int age_s)
  {
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    cloud.resize(point_count);
    for (std::size_t i = 0; i < point_count; ++i)
    {
      cloud[i].x = static_cast<float>(i);
    }
    const auto path = directory_ / file_name;
    pcl::io::savePCDFileASCII(path.string(), cloud);
    std::filesystem::last_write_time(path, std::filesystem::file_time_type::clock::now() - std::chrono::seconds(age_s));
    return path;
  }

  std::filesystem::path directory_;
};
}  // namespace

TEST_F(LoadLatestPointCloudFromFileTest, PicksTheNewestMatchingFile)
{
  writeCloud("pointcloud_old.pcd", 1, 100);
  const auto newest = writeCloud("pointcloud_new.pcd", 2, 10);
  writeCloud("other_newer.pcd", 3, 1);
  std::ofstream(directory_ / "pointcloud_newest.txt") << "not a cloud";

  const auto result = findNewestPcdFile(directory_, "pointcloud");
  ASSERT_TRUE(result.has_value()) << result.error();
  EXPECT_EQ(result.value(), newest);

  const auto any = findNewestPcdFile(directory_, "");
  ASSERT_TRUE(any.has_value()) << any.error();
  EXPECT_EQ(any.value().filename(), "other_newer.pcd");
}

TEST_F(LoadLatestPointCloudFromFileTest, NoMatchOrNoDirectoryFails)
{
  writeCloud("other.pcd", 1, 1);
  EXPECT_FALSE(findNewestPcdFile(directory_, "pointcloud").has_value());
  EXPECT_FALSE(findNewestPcdFile(directory_ / "missing", "").has_value());
}

TEST_F(LoadLatestPointCloudFromFileTest, TickLoadsTheCloudAndSetsTheFrame)
{
  writeCloud("scan_a.pcd", 2, 50);
  const auto newest = writeCloud("scan_b.pcd", 4, 5);

  auto node = std::make_shared<rclcpp::Node>("test_load_latest_point_cloud_from_file");
  auto shared_resources = std::make_shared<moveit_pro::behaviors::BehaviorContext>(node, false);
  BT::NodeConfiguration config;
  config.blackboard = BT::Blackboard::create();
  config.input_ports["directory"] = directory_.string();
  config.input_ports["file_prefix"] = "scan_";
  config.input_ports["frame_id"] = "sensor_frame";
  config.output_ports["point_cloud"] = "{point_cloud}";
  config.output_ports["file_path"] = "{file_path}";

  point_cloud_behaviors::LoadLatestPointCloudFromFile behavior("test", config, shared_resources);
  ASSERT_EQ(behavior.executeTick(), BT::NodeStatus::SUCCESS);
  const auto cloud = config.blackboard->get<sensor_msgs::msg::PointCloud2>("point_cloud");
  EXPECT_EQ(cloud.width * cloud.height, 4u);
  EXPECT_EQ(cloud.header.frame_id, "sensor_frame");
  EXPECT_EQ(config.blackboard->get<std::string>("file_path"), newest.string());
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
