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
 * @brief Reflects a point cloud across a plane, for example so a one-sided scan of a symmetric object
 * can be mirrored to fill the unscanned half before merging.
 *
 * The cloud must be in the frame the plane is expressed in. All non-xyz fields are kept.
 *
 * | Data Port Name | Port Type | Object Type                                   |
 * | -------------- | --------- | --------------------------------------------- |
 * | input_cloud    | Input     | sensor_msgs::msg::PointCloud2                 |
 * | plane_point    | Input     | std::string, a point on the plane as "x;y;z"  |
 * | plane_normal   | Input     | std::string, the plane normal as "x;y;z"      |
 * | output_cloud   | Output    | sensor_msgs::msg::PointCloud2                 |
 */
class MirrorPointCloud : public BT::SyncActionNode
{
public:
  MirrorPointCloud(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};
}  // namespace point_cloud_behaviors
