// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

namespace experimental_behaviors::detail
{
struct JointPositionBounds
{
  bool bounded;
  double minimum;
  double maximum;
};

// Runtime versions expose the same bounds with different member spellings.
template <typename BoundsT>
[[nodiscard]] auto positionBounds(const BoundsT& bounds, int)
    -> decltype(bounds.position_bounded_, bounds.min_position_, bounds.max_position_, JointPositionBounds{})
{
  return { bounds.position_bounded_, bounds.min_position_, bounds.max_position_ };
}

template <typename BoundsT>
[[nodiscard]] auto positionBounds(const BoundsT& bounds, long)
    -> decltype(bounds.position_bounded, bounds.min_position, bounds.max_position, JointPositionBounds{})
{
  return { bounds.position_bounded, bounds.min_position, bounds.max_position };
}
}  // namespace experimental_behaviors::detail
