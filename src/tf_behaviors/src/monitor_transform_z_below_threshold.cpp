#include "tf_behaviors/monitor_transform_z_below_threshold.hpp"

#include <spdlog/spdlog.h>
#include <tf2_ros/buffer.h>

#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

namespace
{
constexpr auto kPortIDTargetFrame = "target_frame_id";
constexpr auto kPortIDSourceFrame = "source_frame_id";
constexpr auto kPortIDThresholdZ = "threshold_z";

inline constexpr auto kDescription = R"(
    <p>
        Returns SUCCESS as soon as the source frame's Z coordinate, expressed in the target
        frame, drops to or below <code>threshold_z</code>. Until then the behavior returns
        RUNNING. On TF lookup failure it also returns RUNNING (the parent Parallel keeps
        ticking, the lookup retries next tick).
    </p>
    <p>
        Use under a Parallel(success_count=1) alongside other stop conditions for an open-loop
        descent — typically an F/T magnitude monitor plus a wall-clock timeout. The threshold
        is the world Z at which the carried object would sit on the target surface (surface Z
        + half the object's vertical extent + clearance), so the descent stops at the surface
        even if the F/T sensor never trips.
    </p>
)";
}  // namespace

namespace tf_behaviors
{

MonitorTransformZBelowThreshold::MonitorTransformZBelowThreshold(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::StatefulActionNode>(name, config, shared_resources)
{
}

BT::PortsList MonitorTransformZBelowThreshold::providedPorts()
{
  return {
    BT::InputPort<std::string>(kPortIDTargetFrame, "world",
                               "Target frame for the TF lookup. The Z coordinate is read in this frame."),
    BT::InputPort<std::string>(kPortIDSourceFrame, "grasp_link",
                               "Source frame whose origin's Z coordinate is monitored."),
    BT::InputPort<double>(kPortIDThresholdZ, 0.0,
                          "Z threshold (m). The behavior returns SUCCESS the first tick "
                          "source_frame_id's Z (in target_frame_id) is <= threshold_z."),
  };
}

BT::KeyValueVector MonitorTransformZBelowThreshold::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "Monitors" },
           { moveit_pro::behaviors::kDescriptionMetadataKey, kDescription } };
}

BT::NodeStatus MonitorTransformZBelowThreshold::onStart()
{
  return onRunning();
}

BT::NodeStatus MonitorTransformZBelowThreshold::onRunning()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(getInput<std::string>(kPortIDTargetFrame),
                                                              getInput<std::string>(kPortIDSourceFrame),
                                                              getInput<double>(kPortIDThresholdZ));
  if (!ports.has_value())
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "Failed to get required value from input data port: " +
                                                                    ports.error());
    return BT::NodeStatus::FAILURE;
  }

  const auto& [target_frame_id, source_frame_id, threshold_z] = ports.value();

  if (!getBehaviorContext()->transform_buffer_ptr->canTransform(target_frame_id, source_frame_id, tf2::TimePointZero))
  {
    return BT::NodeStatus::RUNNING;
  }

  const auto transform =
      getBehaviorContext()->transform_buffer_ptr->lookupTransform(target_frame_id, source_frame_id, tf2::TimePointZero);
  const double z = transform.transform.translation.z;

  if (z <= threshold_z)
  {
    spdlog::info("MonitorTransformZBelowThreshold: {} in {} z={:.4f} <= threshold={:.4f} -> SUCCESS", source_frame_id,
                 target_frame_id, z, threshold_z);
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void MonitorTransformZBelowThreshold::onHalted()
{
  // No-op: nothing persistent to clean up.
}

}  // namespace tf_behaviors
