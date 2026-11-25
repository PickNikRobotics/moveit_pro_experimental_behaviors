#pragma once

#include <behaviortree_cpp/action_node.h>

#include <moveit_pro_behavior_interface/async_behavior_base.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

namespace point_cloud_behaviors
{
/**
 * @brief Removes points that have fewer than `min_neighbors` neighbors within `radius` (Euclidean),
 *        using a PCL KD-tree radius search.
 */
class ExampleRadiusNeighborsFilter : public moveit_pro::behaviors::AsyncBehaviorBase
{
public:
  ExampleRadiusNeighborsFilter(const std::string& name, const BT::NodeConfiguration& config,
                               const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();

protected:
  tl::expected<bool, std::string> doWork() override;

private:
  std::shared_future<tl::expected<bool, std::string>>& getFuture() override
  {
    return future_;
  }

  std::shared_future<tl::expected<bool, std::string>> future_;
};
}  // namespace point_cloud_behaviors
