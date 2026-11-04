// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <Eigen/Core>
#include <string>
#include <vector>

namespace experimental_behaviors::detail
{
// Prefer the legacy vector overload when available; current Runtime accepts an Eigen view of the same storage.
template <typename RobotStateT>
auto setVariablePositions(RobotStateT& state, const std::vector<std::string>& names,
                          const std::vector<double>& positions, int)
    -> decltype(state.setVariablePositions(names, positions), void())
{
  state.setVariablePositions(names, positions);
}

template <typename RobotStateT>
auto setVariablePositions(RobotStateT& state, const std::vector<std::string>& names,
                          const std::vector<double>& positions, long)
    -> decltype(state.setVariablePositions(names, Eigen::Map<const Eigen::VectorXd>(
                                                      positions.data(), static_cast<Eigen::Index>(positions.size()))),
                void())
{
  state.setVariablePositions(names, Eigen::Map<const Eigen::VectorXd>(positions.data(),
                                                                      static_cast<Eigen::Index>(positions.size())));
}
}  // namespace experimental_behaviors::detail
