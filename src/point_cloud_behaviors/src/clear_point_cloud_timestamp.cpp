#include "point_cloud_behaviors/clear_point_cloud_timestamp.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <string>

namespace point_cloud_behaviors
{

ClearPointCloudTimestamp::ClearPointCloudTimestamp(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList ClearPointCloudTimestamp::providedPorts()
{
  return { BT::InputPort<sensor_msgs::msg::PointCloud2>("point_cloud", "{point_cloud}",
                                                        "Point cloud whose header stamp should be zeroed."),
           BT::OutputPort<sensor_msgs::msg::PointCloud2>("stamped_point_cloud", "{stamped_point_cloud}",
                                                         "The same cloud with header.stamp set to zero. The frame_id "
                                                         "and points are unchanged.") };
}

BT::KeyValueVector ClearPointCloudTimestamp::metadata()
{
  return { { "subcategory", "Perception - 3D Point Cloud" },
           { "description", "Zeroes a point cloud's header stamp so TF lookups against it resolve to the latest "
                            "available transforms. Use before UpdatePlanningSceneService on any cloud that is not "
                            "being pushed the instant it was captured, otherwise the octomap updater cannot "
                            "self-filter the moving robot links out of it." } };
}

BT::NodeStatus ClearPointCloudTimestamp::tick()
{
  const auto point_cloud = getInput<sensor_msgs::msg::PointCloud2>("point_cloud");
  if (!point_cloud)
  {
    getBehaviorContext()->logger->publishWarnMessage(name(), "Failed to get required input [point_cloud]: " +
                                                                 point_cloud.error());
    return BT::NodeStatus::FAILURE;
  }

  sensor_msgs::msg::PointCloud2 out_msg = point_cloud.value();
  out_msg.header.stamp.sec = 0;
  out_msg.header.stamp.nanosec = 0;

  setOutput("stamped_point_cloud", out_msg);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace point_cloud_behaviors
