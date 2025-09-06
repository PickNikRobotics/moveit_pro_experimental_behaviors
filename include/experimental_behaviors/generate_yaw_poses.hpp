// BSD-3-Clause
#pragma once

#include <moveit/task_constructor/stages/generate_pose.h>  // brings in MonitoringGenerator
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <queue>

namespace moveit { namespace task_constructor { class SolutionBase; }}

namespace moveit { namespace task_constructor { namespace stages {

// Generator that spawns multiple target poses by rotating a base pose around the Z axis
class GeneratePlanarPoses : public MonitoringGenerator
{
public:
  GeneratePlanarPoses(const std::string& name = "GeneratePlanarPoses");

  void reset() override;
  void onNewSolution(const SolutionBase& s) override;
  bool canCompute() const override;
  void compute() override;

private:
  std::queue<const moveit::task_constructor::SolutionBase*> upstream_solutions_;
};

}}}  // namespace moveit::task_constructor::stages
