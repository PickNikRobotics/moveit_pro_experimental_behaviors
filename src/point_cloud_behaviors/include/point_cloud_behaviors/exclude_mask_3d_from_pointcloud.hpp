#pragma once

#include <behaviortree_cpp/basic_types.h>

#include <moveit_pro_behavior_interface/async_behavior_base.hpp>

namespace point_cloud_behaviors
{
class ExcludeMask3DFromPointCloud : public moveit_pro::behaviors::AsyncBehaviorBase
{
public:
  ExcludeMask3DFromPointCloud(const std::string& name, const BT::NodeConfiguration& config,
                              const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

protected:
  tl::expected<bool, std::string> doWork() override;

private:
  /** @brief Classes derived from AsyncBehaviorBase must implement getFuture() so that it returns a shared_future class member */
  std::shared_future<tl::expected<bool, std::string>>& getFuture() override
  {
    return future_;
  }

  /** @brief Classes derived from AsyncBehaviorBase must have this shared_future as a class member */
  std::shared_future<tl::expected<bool, std::string>> future_;
};
}  // namespace point_cloud_behaviors
