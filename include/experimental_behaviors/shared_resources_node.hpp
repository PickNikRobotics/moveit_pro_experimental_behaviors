// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <memory>
#include <string>

#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

namespace experimental_behaviors
{
/**
 * @brief Provides the constructor's BehaviorContext on both legacy and current Runtime APIs.
 * @details Retains a shared pointer without depending on the Runtime base's protected members or accessors.
 */
template <typename BehaviorTreeNodeT>
class ExperimentalSharedResourcesNode : public moveit_pro::behaviors::SharedResourcesNode<BehaviorTreeNodeT>
{
public:
  ExperimentalSharedResourcesNode(const std::string& name, const BT::NodeConfiguration& config,
                                  const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& behavior_context)
    : moveit_pro::behaviors::SharedResourcesNode<BehaviorTreeNodeT>(name, config, behavior_context)
    , experimental_behavior_context_(behavior_context)
  {
  }

  [[nodiscard]] const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& getExperimentalBehaviorContext() const
  {
    return experimental_behavior_context_;
  }

private:
  std::shared_ptr<moveit_pro::behaviors::BehaviorContext> experimental_behavior_context_;
};
}  // namespace experimental_behaviors
