#include "experimental_behaviors/execute_tracked_trajectory.hpp"
namespace moveit_studio::behaviors
{

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

ExecuteAndTrackJointTrajectory::ExecuteAndTrackJointTrajectory(const std::string& name,
                                                               const BT::NodeConfiguration& config,
                                                               const std::shared_ptr<BehaviorContext>& shared_resources)
  : AsyncBehaviorBase(name, config, shared_resources)
{
}

ExecuteAndTrackJointTrajectory::~ExecuteAndTrackJointTrajectory() = default;

BT::PortsList ExecuteAndTrackJointTrajectory::providedPorts()
{
  return {
    BT::InputPort<std::string>("action_name", "/joint_trajectory_controller/follow_joint_trajectory",
                               "Name of the FollowJointTrajectory action server"),
    BT::InputPort<std::string>("service_name", "/pause_trajectory",
                               "Name of the Trigger service for pausing trajectories"),
    BT::InputPort<trajectory_msgs::msg::JointTrajectory>("joint_trajectory_msg", "",
                                                         "JointTrajectory message to execute"),
    BT::InputPort<double>("goal_position_tolerance", 0.0, "Tolerance for position error (radians)"),
    BT::InputPort<double>("goal_time_tolerance", 0.0, "Tolerance for time error (seconds)"),
    BT::InputPort<double>("goal_duration_tolerance", -1.0,
                          "Time (in seconds) to wait before canceling the goal. Negative means wait indefinitely"),
    BT::InputPort<double>("goal_accept_tolerance", 3.0, "Time (in seconds) to wait for the goal to be accepted"),
    BT::OutputPort<trajectory_msgs::msg::JointTrajectory>("trajectory_remainder", "{joint_trajectory_remainder}",
                                                          "Remaining trajectory after cancellation"),
    BT::OutputPort<double>("time_from_start", "{time_from_start}", "Latest feedback time_from_start in seconds")
  };
}

std::shared_future<tl::expected<bool, std::string>>& ExecuteAndTrackJointTrajectory::getFuture()
{
  return future_;
}

tl::expected<bool, std::string> ExecuteAndTrackJointTrajectory::doWork()
{
  notifyCanHalt();

  auto ports =
      getRequiredInputs(getInput<std::string>("action_name"), getInput<std::string>("service_name"),
                        getInput<trajectory_msgs::msg::JointTrajectory>("joint_trajectory_msg"),
                        getInput<double>("goal_position_tolerance"), getInput<double>("goal_time_tolerance"),
                        getInput<double>("goal_accept_tolerance"), getInput<double>("goal_duration_tolerance"));
  if (!ports)
  {
    return tl::make_unexpected(std::string("Failed to get required input: ") + ports.error());
  }
  auto [action_name, service_name, joint_trajectory, position_tolerance, time_tolerance, acceptance_tolerance,
        duration_tolerance] = ports.value();
  full_trajectory_ = joint_trajectory;

  // Advertise cancellation trigger service
  trigger_service_ = shared_resources_->node->create_service<std_srvs::srv::Trigger>(
      service_name,
      [this](const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) { handleTrigger(res); });

  action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(shared_resources_->node, action_name);
  if (!action_client_->wait_for_action_server(std::chrono::duration<double>(acceptance_tolerance)))
  {
    trigger_service_.reset();
    return tl::make_unexpected("FollowJointTrajectory action server not available.");
  }

  if (full_trajectory_.points.empty())
  {
    return tl::make_unexpected("Empty JointTrajectory Message received, nothing to execute.");
  }
  if (position_tolerance < 0.0 || time_tolerance < 0.0)
  {
    return tl::make_unexpected("Position and time tolerances must be positive or zero.");
  }

  // Build goal
  FollowJointTrajectory::Goal goal;
  goal.trajectory = full_trajectory_;
  for (const auto& jn : goal.trajectory.joint_names)
  {
    control_msgs::msg::JointTolerance tol;
    tol.name = jn;
    tol.position = position_tolerance;
    goal.goal_tolerance.push_back(tol);
  }
  goal.goal_time_tolerance = rclcpp::Duration::from_seconds(time_tolerance);

  // Send goal with feedback callback
  typename rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions options;
  options.feedback_callback = [this](auto, std::shared_ptr<const FollowJointTrajectory::Feedback> fb) {
    processFeedback(fb);
  };
  auto future_goal_handle = action_client_->async_send_goal(goal, options);
  if (future_goal_handle.wait_for(std::chrono::duration<double>(acceptance_tolerance)) == std::future_status::timeout)
  {
    trigger_service_.reset();
    return tl::make_unexpected("Timed out waiting for goal to be accepted.");
  }
  goal_handle_ = future_goal_handle.get();
  if (!goal_handle_)
  {
    trigger_service_.reset();
    return tl::make_unexpected("Goal request was rejected by the server.");
  }

  // Wait for result (with optional timeout)
  auto future_result = action_client_->async_get_result(goal_handle_);
  if (duration_tolerance >= 0.0)
  {
    if (future_result.wait_for(std::chrono::duration<double>(duration_tolerance)) == std::future_status::timeout)
    {
      action_client_->async_cancel_goal(goal_handle_);
      trigger_service_.reset();
      return tl::make_unexpected("Timed out waiting for result.");
    }
  }
  auto wrapped_result = future_result.get();

  // Handle completion or cancellation
  switch (wrapped_result.code)
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      trigger_service_.reset();
      return true;

    case rclcpp_action::ResultCode::CANCELED:
      trigger_service_.reset();
      if (triggered_)
      {
        auto rem = computeRemainderTrajectory();
        setOutput<trajectory_msgs::msg::JointTrajectory>("trajectory_remainder", rem);
        return tl::make_unexpected("Trajectory halted");
      }
      else
      {
        return tl::make_unexpected("Trajectory canceled unexpectedly");
      }

    case rclcpp_action::ResultCode::ABORTED:
      trigger_service_.reset();
      return tl::make_unexpected("Action aborted: " + wrapped_result.result->error_string);

    default:
      trigger_service_.reset();
      return tl::make_unexpected("Unknown action result code");
  }
}

tl::expected<void, std::string> ExecuteAndTrackJointTrajectory::doHalt()
{
  triggered_ = true;
  if (goal_handle_)
  {
    try
    {
      action_client_->async_cancel_goal(goal_handle_);
    }
    catch (const rclcpp_action::exceptions::UnknownGoalHandleError& e)
    {
      RCLCPP_WARN(rclcpp::get_logger("ExecuteAndTrackJointTrajectory"), "doHalt cancel failed: %s", e.what());
    }
    goal_handle_.reset();
  }
  trigger_service_.reset();
  return {};
}

void ExecuteAndTrackJointTrajectory::handleTrigger(const std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  triggered_ = true;
  if (goal_handle_)
  {
    try
    {
      action_client_->async_cancel_goal(goal_handle_);
    }
    catch (const rclcpp_action::exceptions::UnknownGoalHandleError& e)
    {
      RCLCPP_WARN(rclcpp::get_logger("ExecuteAndTrackJointTrajectory"), "Failed to cancel goal: %s", e.what());
    }
    goal_handle_.reset();
  }
  res->success = true;
  res->message = "Trajectory cancellation triggered";
}

void ExecuteAndTrackJointTrajectory::processFeedback(
    const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback)
{
  last_feedback_time_ = feedback->desired.time_from_start.sec + feedback->desired.time_from_start.nanosec * 1e-9;
  setOutput<double>("time_from_start", last_feedback_time_);
}

trajectory_msgs::msg::JointTrajectory ExecuteAndTrackJointTrajectory::computeRemainderTrajectory()
{
  trajectory_msgs::msg::JointTrajectory new_traj;
  new_traj.joint_names = full_trajectory_.joint_names;

  size_t start_index = 0;
  for (; start_index < full_trajectory_.points.size(); ++start_index)
  {
    double t = full_trajectory_.points[start_index].time_from_start.sec +
               full_trajectory_.points[start_index].time_from_start.nanosec * 1e-9;
    if (t >= last_feedback_time_)
      break;
  }
  if (start_index >= full_trajectory_.points.size())
    return new_traj;

  double offset = full_trajectory_.points[start_index].time_from_start.sec +
                  full_trajectory_.points[start_index].time_from_start.nanosec * 1e-9;
  for (size_t i = start_index; i < full_trajectory_.points.size(); ++i)
  {
    auto pt = full_trajectory_.points[i];
    double orig = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9;
    double adjusted = orig - offset;
    pt.time_from_start.sec = static_cast<uint32_t>(adjusted);
    pt.time_from_start.nanosec = static_cast<uint32_t>((adjusted - pt.time_from_start.sec) * 1e9);
    new_traj.points.push_back(pt);
  }
  return new_traj;
}

}  // namespace moveit_studio::behaviors
