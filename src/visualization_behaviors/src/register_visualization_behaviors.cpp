// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "visualization_behaviors/register_visualization_behaviors.hpp"

#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>

#include "visualization_behaviors/clear_stale_contour_markers.hpp"
#include "visualization_behaviors/publish_marker_array.hpp"
#include "visualization_behaviors/publish_modify_markers.hpp"

namespace visualization_behaviors
{
void registerVisualizationBehaviors(BT::BehaviorTreeFactory& factory,
                                    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
{
  moveit_pro::behaviors::registerBehavior<ClearStaleContourMarkers>(factory, "ClearStaleContourMarkers",
                                                                    shared_resources);
  moveit_pro::behaviors::registerBehavior<PublishMarkerArray>(factory, "PublishMarkerArray", shared_resources);
  moveit_pro::behaviors::registerBehavior<PublishModifyMarkers>(factory, "PublishModifyMarkers", shared_resources);
}
}  // namespace visualization_behaviors
