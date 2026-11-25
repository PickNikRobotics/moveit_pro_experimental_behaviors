// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "point_cloud_behaviors/register_point_cloud_behaviors.hpp"

#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>

#include "point_cloud_behaviors/clear_point_cloud_timestamp.hpp"
#include "point_cloud_behaviors/count_points_in_cloud.hpp"
#include "point_cloud_behaviors/crop_point_cloud_to_camera_frustum.hpp"
#include "point_cloud_behaviors/exclude_mask_3d_from_pointcloud.hpp"
#include "point_cloud_behaviors/get_closest_point_pose.hpp"
#include "point_cloud_behaviors/load_latest_point_cloud_from_file.hpp"
#include "point_cloud_behaviors/mirror_pointcloud.hpp"
#include "point_cloud_behaviors/pcl_nn_filter.hpp"
#include "point_cloud_behaviors/poisson_reconstruct.hpp"
#include "point_cloud_behaviors/reconstruct_surface.hpp"

namespace point_cloud_behaviors
{
void registerPointCloudBehaviors(BT::BehaviorTreeFactory& factory,
                                 const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
{
  moveit_pro::behaviors::registerBehavior<ClearPointCloudTimestamp>(factory, "ClearPointCloudTimestamp",
                                                                    shared_resources);
  moveit_pro::behaviors::registerBehavior<CountPointsInCloud>(factory, "CountPointsInCloud", shared_resources);
  moveit_pro::behaviors::registerBehavior<CropPointCloudToCameraFrustum>(factory, "CropPointCloudToCameraFrustum",
                                                                         shared_resources);
  moveit_pro::behaviors::registerBehavior<ExcludeMask3DFromPointCloud>(factory, "ExcludeMask3DFromPointCloud",
                                                                       shared_resources);
  moveit_pro::behaviors::registerBehavior<GetClosestPointPose>(factory, "GetClosestPointPose", shared_resources);
  moveit_pro::behaviors::registerBehavior<LoadLatestPointCloudFromFile>(factory, "LoadLatestPointCloudFromFile",
                                                                        shared_resources);
  moveit_pro::behaviors::registerBehavior<MirrorPointCloud>(factory, "MirrorPointCloud");
  moveit_pro::behaviors::registerBehavior<ExampleRadiusNeighborsFilter>(factory, "PclNnFilter", shared_resources);
  moveit_pro::behaviors::registerBehavior<PoissonReconstruct>(factory, "PoissonReconstruct");
  moveit_pro::behaviors::registerBehavior<ReconstructSurface>(factory, "ReconstructSurface");
}
}  // namespace point_cloud_behaviors
