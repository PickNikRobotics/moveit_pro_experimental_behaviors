#pragma once

#include <experimental_behaviors/setup_mtc_debug_stage.hpp>
#include <experimental_behaviors/generate_yaw_poses.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

namespace moveit_studio::behaviors {

class SetupMTCGeneratePlanarPoses final : public MTCStageHelper<SetupMTCGeneratePlanarPoses>
{
  MTC_STAGE_BEHAVIOR_INTERFACE(SetupMTCGeneratePlanarPoses,
                               "Generate Yaw Poses",
                               "to spawn multiple target poses by sampling yaw about the +Z axis.")

public:
  static constexpr auto kPortIDPose = "pose";  // base pose (frame + xyz + orientation)
  static constexpr auto kPortIDNumPoses = "num_poses";  // number of yaw samples
  static constexpr auto kPortIDStartYaw = "start_yaw";  // rad
  static constexpr auto kPortIDSweep = "sweep";         // rad

  static BT::PortsList providedPorts()
  {
    return MTCCommonPorts::mergePorts({
      BT::InputPort<geometry_msgs::msg::PoseStamped>(kPortIDPose, "{pose}", "Base pose to sample from (frame required)"),
      BT::InputPort<unsigned int>(kPortIDNumPoses, 8, "Number of yaw samples to generate (>0)"),
      BT::InputPort<double>(kPortIDStartYaw, 0.0, "Start yaw offset [rad] relative to base"),
      BT::InputPort<double>(kPortIDSweep, 6.283185307179586, "Total yaw sweep [rad]")
    });
  }

  tl::expected<std::unique_ptr<moveit::task_constructor::Stage>, std::string>
  createStage([[maybe_unused]] const StageInputs& inputs)
  {
    const auto custom_ports = getRequiredInputs(
      getInput<geometry_msgs::msg::PoseStamped>(kPortIDPose),
      getInput<unsigned int>(kPortIDNumPoses),
      getInput<double>(kPortIDStartYaw),
      getInput<double>(kPortIDSweep)
    );

    if (!custom_ports.has_value())
      return tl::make_unexpected("Failed to get required input: " + custom_ports.error());

    const auto& [pose, num_poses, start_yaw, sweep] = custom_ports.value();
    if (num_poses == 0)
      return tl::make_unexpected("num_poses must be > 0");

    auto stage = std::make_unique<moveit::task_constructor::stages::GeneratePlanarPoses>(kDefaultStageName);
    auto& p = stage->properties();
    p.set("pose", pose);
    p.set("num_poses", num_poses);
    p.set("start_yaw", start_yaw);
    p.set("sweep", sweep);

    return stage;
  }
};

}  // namespace moveit_studio::behaviors
