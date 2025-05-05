#pragma once

#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <moveit_studio_behavior_interface/async_behavior_base.hpp>
#include <moveit_studio_behavior_interface/get_optional_ports.hpp>
#include <moveit_studio_behavior_interface/get_required_ports.hpp>
#include <tl_expected/expected.hpp>

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

namespace moveit_studio::behaviors
{

class ExecuteAndTrackJointTrajectory : public AsyncBehaviorBase
{
public:
  ExecuteAndTrackJointTrajectory(const std::string& name, const BT::NodeConfiguration& config,
                                 const std::shared_ptr<BehaviorContext>& shared_resources);
  ~ExecuteAndTrackJointTrajectory() override;

  static BT::PortsList providedPorts();

  std::shared_future<tl::expected<bool, std::string>>& getFuture() override;

protected:
  tl::expected<bool, std::string> doWork() override;
  tl::expected<void, std::string> doHalt() override;

private:
  void handleTrigger(const std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void processFeedback(const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback);
  trajectory_msgs::msg::JointTrajectory computeRemainderTrajectory();

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_service_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
  rclcpp_action::ClientGoalHandle<FollowJointTrajectory>::SharedPtr goal_handle_;
  trajectory_msgs::msg::JointTrajectory full_trajectory_;
  double last_feedback_time_{ 0.0 };
  std::atomic<bool> triggered_{ false };
  std::shared_future<tl::expected<bool, std::string>> future_;
};

}  // namespace moveit_studio::behaviors
