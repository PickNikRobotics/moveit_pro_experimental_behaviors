// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "geometry_behaviors/register_geometry_behaviors.hpp"

#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>

#include "geometry_behaviors/check_bounding_box_in_bounds.hpp"
#include "geometry_behaviors/compute_camera_pose_from_tag.hpp"
#include "geometry_behaviors/compute_relative_axis_dot.hpp"
#include "geometry_behaviors/create_pose_stamped_grid.hpp"
#include "geometry_behaviors/express_world_vector_in_link_frame.hpp"
#include "geometry_behaviors/generate_axis_rotated_candidates.hpp"
#include "geometry_behaviors/get_bounding_box_face_pose.hpp"
#include "geometry_behaviors/is_pose_in_box.hpp"
#include "geometry_behaviors/orient_pose_axis_to_reference_axis.hpp"
#include "geometry_behaviors/orient_pose_plane_to_reference_plane.hpp"

namespace geometry_behaviors
{
void registerGeometryBehaviors(BT::BehaviorTreeFactory& factory,
                               const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
{
  moveit_pro::behaviors::registerBehavior<CheckBoundingBoxInBounds>(factory, "CheckBoundingBoxInBounds",
                                                                    shared_resources);
  moveit_pro::behaviors::registerBehavior<ComputeCameraPoseFromTag>(factory, "ComputeCameraPoseFromTag",
                                                                    shared_resources);
  moveit_pro::behaviors::registerBehavior<ComputeRelativeAxisDot>(factory, "ComputeRelativeAxisDot", shared_resources);
  moveit_pro::behaviors::registerBehavior<CreatePoseStampedGrid>(factory, "CreatePoseStampedGrid", shared_resources);
  moveit_pro::behaviors::registerBehavior<ExpressWorldVectorInLinkFrame>(factory, "ExpressWorldVectorInLinkFrame");
  moveit_pro::behaviors::registerBehavior<GenerateAxisRotatedCandidates>(factory, "GenerateAxisRotatedCandidates",
                                                                         shared_resources);
  moveit_pro::behaviors::registerBehavior<GetBoundingBoxFacePose>(factory, "GetBoundingBoxFacePose", shared_resources);
  moveit_pro::behaviors::registerBehavior<IsPoseInBox>(factory, "IsPoseInBox", shared_resources);
  moveit_pro::behaviors::registerBehavior<OrientPoseAxisToReferenceAxis>(factory, "OrientPoseAxisToReferenceAxis",
                                                                         shared_resources);
  moveit_pro::behaviors::registerBehavior<OrientPosePlaneToReferencePlane>(factory, "OrientPosePlaneToReferencePlane",
                                                                           shared_resources);
}
}  // namespace geometry_behaviors
