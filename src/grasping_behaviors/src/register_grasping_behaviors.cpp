// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "grasping_behaviors/register_grasping_behaviors.hpp"

#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>

#include "grasping_behaviors/determine_pick_alignment.hpp"
#include "grasping_behaviors/extract_graspable_object_primitive.hpp"
#include "grasping_behaviors/filter_graspable_objects.hpp"
#include "grasping_behaviors/generate_grasp_candidates.hpp"
#include "grasping_behaviors/generate_place_candidates.hpp"

namespace grasping_behaviors
{
void registerGraspingBehaviors(BT::BehaviorTreeFactory& factory,
                               const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
{
  moveit_pro::behaviors::registerBehavior<DeterminePickAlignment>(factory, "DeterminePickAlignment");
  moveit_pro::behaviors::registerBehavior<ExtractGraspableObjectPrimitive>(factory, "ExtractGraspableObjectPrimitive");
  moveit_pro::behaviors::registerBehavior<FilterGraspableObjects>(factory, "FilterGraspableObjects", shared_resources);
  moveit_pro::behaviors::registerBehavior<GenerateGraspCandidates>(factory, "GenerateGraspCandidates", shared_resources);
  moveit_pro::behaviors::registerBehavior<GeneratePlaceCandidates>(factory, "GeneratePlaceCandidates", shared_resources);
}
}  // namespace grasping_behaviors
