// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "image_behaviors/register_image_behaviors.hpp"

#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>

#include "image_behaviors/find_mask_index_for_point.hpp"
#include "image_behaviors/find_points_in_masks.hpp"
#include "image_behaviors/get_highest_confidence_mask.hpp"
#include "image_behaviors/intersect_pixel_rays_with_plane.hpp"
#include "image_behaviors/mark_points_on_image.hpp"
#include "image_behaviors/rotate_and_crop_object_image.hpp"
#include "image_behaviors/save_depth_image_to_file.hpp"

namespace image_behaviors
{
void registerImageBehaviors(BT::BehaviorTreeFactory& factory,
                            const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
{
  moveit_pro::behaviors::registerBehavior<FindMaskIndexForPoint>(factory, "FindMaskIndexForPoint", shared_resources);
  moveit_pro::behaviors::registerBehavior<FindPointsInMasks>(factory, "FindPointsInMasks", shared_resources);
  moveit_pro::behaviors::registerBehavior<GetHighestConfidenceMask>(factory, "GetHighestConfidenceMask",
                                                                    shared_resources);
  moveit_pro::behaviors::registerBehavior<IntersectPixelRaysWithPlane>(factory, "IntersectPixelRaysWithPlane",
                                                                       shared_resources);
  moveit_pro::behaviors::registerBehavior<MarkPointsOnImage>(factory, "MarkPointsOnImage", shared_resources);
  moveit_pro::behaviors::registerBehavior<RotateAndCropObjectImage>(factory, "RotateAndCropObjectImage",
                                                                    shared_resources);
  moveit_pro::behaviors::registerBehavior<SaveDepthImageToFile>(factory, "SaveDepthImageToFile", shared_resources);
}
}  // namespace image_behaviors
