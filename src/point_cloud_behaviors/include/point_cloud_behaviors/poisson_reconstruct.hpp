// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <behaviortree_cpp/action_node.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace point_cloud_behaviors
{
/**
 * @brief Screened Poisson surface reconstruction with distance trimming.
 *
 * Estimates oriented normals, builds a watertight Poisson mesh that bridges bounded gaps, then drops mesh
 * vertices farther than trim_distance from any measured point. See docs/AGENT_COMMENTS.md.
 *
 * Output is a PointXYZRGB cloud: measured points in scanned_color, filled points in filled_color.
 *
 * | Data Port Name | Port Type | Object Type                                              |
 * | -------------- | --------- | -------------------------------------------------------- |
 * | input_cloud    | Input     | sensor_msgs::msg::PointCloud2                            |
 * | poisson_depth  | Input     | int, octree depth (higher is finer and slower)           |
 * | normal_radius  | Input     | double, normal estimation radius in meters               |
 * | trim_distance  | Input     | double, meters; smaller keeps closer to the data         |
 * | fill_threshold | Input     | double, meters from a measured point to count as filled  |
 * | scanned_color  | Input     | std::string, "r;g;b" of measured points                  |
 * | filled_color   | Input     | std::string, "r;g;b" of filled points                    |
 * | output_cloud   | Output    | sensor_msgs::msg::PointCloud2 (PointXYZRGB)              |
 */
class PoissonReconstruct : public BT::SyncActionNode
{
public:
  PoissonReconstruct(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};
}  // namespace point_cloud_behaviors
