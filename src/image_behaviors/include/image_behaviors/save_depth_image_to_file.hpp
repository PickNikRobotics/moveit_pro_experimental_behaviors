#pragma once

#include <moveit_pro_behavior_interface/async_behavior_base.hpp>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <string>

namespace image_behaviors
{

/**
 * @brief Saves a depth image (16UC1 or 32FC1) to a PNG file, preserving the original encoding.
 *
 * @details The built-in SaveImageToFile converts to BGR8, which destroys depth data.
 * This behavior writes depth images directly as 16-bit PNGs.
 *
 * | Data Port Name | Port Type | Object Type                    |
 * | -------------- | --------- | ------------------------------ |
 * | file_path      | input     | std::string                    |
 * | file_prefix    | input     | std::string                    |
 * | image          | input     | sensor_msgs::msg::Image        |
 */
class SaveDepthImageToFile final : public moveit_pro::behaviors::AsyncBehaviorBase
{
public:
  SaveDepthImageToFile(const std::string& name, const BT::NodeConfiguration& config,
                       const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();

private:
  tl::expected<bool, std::string> doWork() override;

  std::shared_future<tl::expected<bool, std::string>>& getFuture() override
  {
    return future_;
  }

  std::shared_future<tl::expected<bool, std::string>> future_;
};

}  // namespace image_behaviors
