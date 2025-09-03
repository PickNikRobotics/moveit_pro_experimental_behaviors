// BSD-3-Clause
#pragma once

#include <moveit/task_constructor/stages/monitoring_generator.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace moveit { namespace task_constructor { namespace stages {

// Generator that spawns multiple target poses by rotating a base pose around the Z axis
class GenerateYawPoses : public MonitoringGenerator
{
public:
  GenerateYawPoses(const std::string& name = "GenerateYawPoses");

  void reset() override;
  void onNewSolution(const SolutionBase& s) override;
  bool canCompute() const override;
  void compute() override;

private:
  utils::SolutionBaseQueue upstream_solutions_;
};

}}}  // namespace moveit::task_constructor::stages

