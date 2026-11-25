// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <controller_manager_msgs/srv/set_hardware_component_state.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <moveit_pro_behavior_interface/service_client_behavior_base.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

namespace hardware_behaviors
{
using SetHardwareComponentStateSrv = controller_manager_msgs::srv::SetHardwareComponentState;

/**
 * @brief Drives a `ros2_control` hardware component through a lifecycle transition by calling the controller manager's
 * `set_hardware_component_state` service.
 *
 * @details This is the Behavior equivalent of `ros2 control set_hardware_component_state <name> <state>`. It is needed
 * on any recovery path where the hardware itself went down, not just its controllers: when a driver's `read()` or
 * `write()` returns an error, `ros2_control` error-transitions the component out of the active state, and no amount of
 * `SwitchController` will bring it back.
 *
 * The controller manager works out the transition chain, so an unconfigured component can be taken straight to
 * `active` in one call. Requesting the state a component is already in is a no-op that reports failure, so wrap the
 * defensive `inactive` step of a reset sequence in a `ForceSuccess` decorator.
 *
 * | Data Port Name | Port Type | Object Type |
 * | -------------- | --------- | ----------- |
 * | service_name   | Input     | std::string |
 * | hardware_name  | Input     | std::string |
 * | target_state   | Input     | std::string |
 * | timeout        | Input     | double      |
 */
class SetHardwareComponentState final
  : public moveit_pro::behaviors::ServiceClientBehaviorBase<SetHardwareComponentStateSrv>
{
public:
  SetHardwareComponentState(const std::string& name, const BT::NodeConfiguration& config,
                            const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

private:
  tl::expected<std::string, std::string> getServiceName() override;

  tl::expected<SetHardwareComponentStateSrv::Request, std::string> createRequest() override;

  tl::expected<bool, std::string> processResponse(const SetHardwareComponentStateSrv::Response& response) override;

  tl::expected<std::chrono::duration<double>, std::string> getResponseTimeout() override;

  /** @brief Classes derived from AsyncBehaviorBase must implement getFuture() so that it returns a shared_future class
   * member. */
  std::shared_future<tl::expected<bool, std::string>>& getFuture() override
  {
    return future_;
  }

  /** @brief Classes derived from AsyncBehaviorBase must have this shared_future as a class member. */
  std::shared_future<tl::expected<bool, std::string>> future_;
};
}  // namespace hardware_behaviors
