// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "conversion_behaviors/register_conversion_behaviors.hpp"

#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>

#include "conversion_behaviors/create_vector_of_doubles.hpp"
#include "conversion_behaviors/double_to_string.hpp"
#include "conversion_behaviors/string_to_double.hpp"

namespace conversion_behaviors
{
void registerConversionBehaviors(
    BT::BehaviorTreeFactory& factory,
    [[maybe_unused]] const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
{
  moveit_pro::behaviors::registerBehavior<StringToDouble>(factory, "StringToDouble");
  moveit_pro::behaviors::registerBehavior<DoubleToString>(factory, "DoubleToString");
  moveit_pro::behaviors::registerBehavior<CreateVectorOfDoubles>(factory, "CreateVectorOfDoubles");
}
}  // namespace conversion_behaviors
