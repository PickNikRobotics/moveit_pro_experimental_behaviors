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
 * @brief Surface-following hole fill via Moving Least Squares (PCL MLS).
 *
 * Fits a local polynomial surface around each point and resamples on it, which smooths noise and fills
 * gaps that lie on the surface. Unlike a convex hull it never bridges concavities, so a non-convex object
 * keeps its shape.
 *
 * | Data Port Name      | Port Type | Object Type                                                          |
 * | ------------------- | --------- | -------------------------------------------------------------------- |
 * | input_cloud         | Input     | sensor_msgs::msg::PointCloud2                                        |
 * | search_radius       | Input     | double, MLS neighborhood radius in meters                            |
 * | polynomial_order    | Input     | int, local polynomial order                                          |
 * | upsampling_method   | Input     | std::string: voxel_grid_dilation, sample_local_plane or none         |
 * | voxel_size          | Input     | double, voxel size in meters for voxel_grid_dilation                 |
 * | dilation_iterations | Input     | int, dilation passes; more fills bigger gaps but can over-fill       |
 * | output_cloud        | Output    | sensor_msgs::msg::PointCloud2 (PointXYZ; color is dropped)           |
 */
class ReconstructSurface : public BT::SyncActionNode
{
public:
  ReconstructSurface(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};
}  // namespace point_cloud_behaviors
