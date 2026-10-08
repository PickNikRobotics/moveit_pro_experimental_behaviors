// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <gtest/gtest.h>

#include <experimental_behaviors/localization_behaviors/scan_match_residual_from_topics.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "localization_test_helpers.hpp"

#include <cmath>
#include <limits>

namespace
{
using experimental_behaviors::ScanMatchResidualFromTopics;

constexpr auto kMapTopic = "/localization_behaviors_test/map";
constexpr auto kScanTopic = "/localization_behaviors_test/scan";
constexpr auto kMaxRangeScanTopic = "/localization_behaviors_test/max_range_scan";
constexpr double kResolution = 0.1;
constexpr std::uint32_t kCells = 20;
// The sensor sits this far forward of the robot frame.
constexpr double kSensorForward = 0.4;

nav_msgs::msg::OccupancyGrid makeRoom()
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.info.resolution = kResolution;
  grid.info.width = kCells;
  grid.info.height = kCells;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(kCells * kCells, 0);
  for (std::uint32_t i = 0; i < kCells; ++i)
  {
    grid.data[i] = grid.data[(kCells - 1) * kCells + i] = 100;
    grid.data[i * kCells] = grid.data[i * kCells + kCells - 1] = 100;
  }
  return grid;
}

// From a robot at the room centre facing +X: beams east, north and west, measured from the sensor.
sensor_msgs::msg::LaserScan makeScan()
{
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "test_laser";
  scan.angle_min = 0.0F;
  scan.angle_increment = static_cast<float>(M_PI / 2.0);
  scan.range_min = 0.05F;
  scan.range_max = 10.0F;
  scan.ranges = { static_cast<float>(0.95 - kSensorForward), 0.95F, static_cast<float>(0.95 + kSensorForward) };
  return scan;
}

// Every beam is a max-range return, so no beam has an endpoint.
sensor_msgs::msg::LaserScan makeMaxRangeScan()
{
  auto scan = makeScan();
  scan.ranges.assign(scan.ranges.size(), scan.range_max);
  return scan;
}

geometry_msgs::msg::PoseStamped makePose(double x, const std::string& frame = "map")
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame;
  pose.pose.position.x = x;
  pose.pose.position.y = 1.0;
  pose.pose.orientation.w = 1.0;
  return pose;
}

class ScanMatchResidualFromTopicsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    geometry_msgs::msg::TransformStamped sensor;
    sensor.header.frame_id = "test_robot";
    sensor.child_frame_id = "test_laser";
    sensor.transform.translation.x = kSensorForward;
    sensor.transform.rotation.w = 1.0;
    spun.context->transform_buffer_ptr->setTransform(sensor, "test", true);

    map_publisher = spun.node->create_publisher<nav_msgs::msg::OccupancyGrid>(
        kMapTopic, rclcpp::QoS{ rclcpp::KeepLast(1) }.reliable().transient_local());
    map_publisher->publish(makeRoom());
    scan_publisher = spun.node->create_publisher<sensor_msgs::msg::LaserScan>(kScanTopic, rclcpp::SensorDataQoS());
    max_range_scan_publisher =
        spun.node->create_publisher<sensor_msgs::msg::LaserScan>(kMaxRangeScanTopic, rclcpp::SensorDataQoS());
    scan_timer = spun.node->create_wall_timer(std::chrono::milliseconds{ 50 }, [this] {
      scan_publisher->publish(makeScan());
      max_range_scan_publisher->publish(makeMaxRangeScan());
    });
  }

  BT::NodeStatus run(const geometry_msgs::msg::PoseStamped& pose, double min_fraction = 0.9,
                     double inlier_distance = 0.05)
  {
    config = experimental_behaviors::test::makeConfig();
    config.blackboard->set("pose", pose);
    config.blackboard->set("min_inlier_fraction", min_fraction);
    config.blackboard->set("inlier_distance", inlier_distance);
    config.input_ports["pose"] = "{pose}";
    config.input_ports["robot_frame_id"] = robot_frame;
    config.input_ports["scan_topic"] = scan_topic;
    config.input_ports["map_topic"] = map_topic;
    config.input_ports["inlier_distance"] = "{inlier_distance}";
    config.input_ports["min_inlier_fraction"] = "{min_inlier_fraction}";
    config.input_ports["message_timeout_sec"] = message_timeout;
    for (const auto* port : { "inlier_fraction", "median_residual", "beams_used" })
    {
      config.output_ports[port] = std::string{ "{" } + port + "}";
    }
    ScanMatchResidualFromTopics behavior{ "ScanMatchResidualFromTopics", config, spun.context };
    return experimental_behaviors::test::tickUntilDone(behavior);
  }

  experimental_behaviors::test::SpunContext spun;
  BT::NodeConfiguration config;
  std::string robot_frame = "test_robot";
  std::string scan_topic = kScanTopic;
  std::string map_topic = kMapTopic;
  std::string message_timeout = "3.0";
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr max_range_scan_publisher;
  rclcpp::TimerBase::SharedPtr scan_timer;
};
}  // namespace

TEST_F(ScanMatchResidualFromTopicsTest, PassesAtTheTruePoseThroughTheSensorOffset)
{
  EXPECT_EQ(run(makePose(1.0)), BT::NodeStatus::SUCCESS);
  EXPECT_DOUBLE_EQ(config.blackboard->get<double>("inlier_fraction"), 1.0);
  EXPECT_EQ(config.blackboard->get<int>("beams_used"), 3);
  EXPECT_NEAR(config.blackboard->get<double>("median_residual"), 0.0, 1e-6);
}

TEST_F(ScanMatchResidualFromTopicsTest, FailsAtAWrongPoseButStillOutputsTheScore)
{
  EXPECT_EQ(run(makePose(0.7)), BT::NodeStatus::FAILURE);
  EXPECT_LT(config.blackboard->get<double>("inlier_fraction"), 0.9);
  EXPECT_EQ(config.blackboard->get<int>("beams_used"), 3);
}

TEST_F(ScanMatchResidualFromTopicsTest, AZeroThresholdPassesAnyScoredPose)
{
  EXPECT_EQ(run(makePose(0.7), 0.0), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(config.blackboard->get<int>("beams_used"), 3);
}

TEST_F(ScanMatchResidualFromTopicsTest, FailsWhenThePoseIsNotInTheMapFrame)
{
  EXPECT_EQ(run(makePose(1.0, "odom")), BT::NodeStatus::FAILURE);
}

TEST_F(ScanMatchResidualFromTopicsTest, RejectsAnOutOfRangeFraction)
{
  EXPECT_EQ(run(makePose(1.0), -0.5), BT::NodeStatus::FAILURE);
  EXPECT_EQ(run(makePose(1.0), 1.5), BT::NodeStatus::FAILURE);
}

TEST_F(ScanMatchResidualFromTopicsTest, RejectsANaNFraction)
{
  EXPECT_EQ(run(makePose(0.7), std::numeric_limits<double>::quiet_NaN()), BT::NodeStatus::FAILURE);
}

TEST_F(ScanMatchResidualFromTopicsTest, RejectsAnInfiniteInlierDistance)
{
  EXPECT_EQ(run(makePose(0.7), 0.9, std::numeric_limits<double>::infinity()), BT::NodeStatus::FAILURE);
}

TEST_F(ScanMatchResidualFromTopicsTest, RejectsANonFinitePose)
{
  // A zero threshold would pass any scored pose, so only the pose check can fail this run.
  auto pose = makePose(1.0);
  pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(run(pose, 0.0), BT::NodeStatus::FAILURE);
}

TEST_F(ScanMatchResidualFromTopicsTest, FailsWithoutTheSensorTransform)
{
  robot_frame = "test_unknown_robot";
  EXPECT_EQ(run(makePose(1.0)), BT::NodeStatus::FAILURE);
}

TEST_F(ScanMatchResidualFromTopicsTest, FailsWhenNoBeamIsUsable)
{
  scan_topic = kMaxRangeScanTopic;
  EXPECT_EQ(run(makePose(1.0), 0.0), BT::NodeStatus::FAILURE);
  EXPECT_EQ(config.blackboard->get<int>("beams_used"), 0);
}

TEST_F(ScanMatchResidualFromTopicsTest, FailsWhenNoMapArrives)
{
  map_topic = "/localization_behaviors_test/no_map";
  message_timeout = "0.3";
  EXPECT_EQ(run(makePose(1.0)), BT::NodeStatus::FAILURE);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
