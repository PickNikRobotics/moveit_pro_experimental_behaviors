#include "planning_scene_behaviors/inflate_octomap.hpp"

#include <cmath>
#include <memory>
#include <vector>

#include <octomap/OcTree.h>
#include <octomap/octomap_types.h>
#include <octomap_msgs/conversions.h>

namespace planning_scene_behaviors
{

InflateOctomap::InflateOctomap(const std::string& name, const BT::NodeConfiguration& config,
                               const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList InflateOctomap::providedPorts()
{
  return {
    BT::InputPort<moveit_msgs::msg::PlanningScene>("planning_scene", "{planning_scene}",
                                                   "Input planning scene containing the octomap to inflate."),
    BT::InputPort<double>("inflation_radius", 0.05, "Radius (meters) by which to inflate occupied voxels."),
    BT::OutputPort<moveit_msgs::msg::PlanningScene>("inflated_scene", "{inflated_scene}",
                                                    "Planning scene with the inflated octomap."),
  };
}

BT::KeyValueVector InflateOctomap::metadata()
{
  return { { "subcategory", "Planning Scene" },
           { "description", "Inflates the occupied voxels in the planning scene's octomap by a given radius." } };
}

BT::NodeStatus InflateOctomap::tick()
{
  const auto planning_scene_input = getInput<moveit_msgs::msg::PlanningScene>("planning_scene");
  if (!planning_scene_input)
  {
    getBehaviorContext()->logger->publishWarnMessage(name(), "Failed to get required input [planning_scene]: " +
                                                                 planning_scene_input.error());
    return BT::NodeStatus::FAILURE;
  }

  const auto inflation_radius_input = getInput<double>("inflation_radius");
  if (!inflation_radius_input)
  {
    getBehaviorContext()->logger->publishWarnMessage(name(), "Failed to get required input [inflation_radius]: " +
                                                                 inflation_radius_input.error());
    return BT::NodeStatus::FAILURE;
  }

  const double inflation_radius = inflation_radius_input.value();
  if (inflation_radius <= 0.0)
  {
    getBehaviorContext()->logger->publishWarnMessage(name(), "inflation_radius must be > 0.0");
    return BT::NodeStatus::FAILURE;
  }

  const moveit_msgs::msg::PlanningScene& scene = planning_scene_input.value();
  const octomap_msgs::msg::Octomap& octomap_msg = scene.world.octomap.octomap;

  if (octomap_msg.data.empty())
  {
    getBehaviorContext()->logger->publishWarnMessage(name(), "Planning scene octomap is empty, nothing to inflate.");
    return BT::NodeStatus::FAILURE;
  }

  // Deserialize the octomap message into an OcTree
  std::unique_ptr<octomap::AbstractOcTree> abstract_tree(octomap_msgs::msgToMap(octomap_msg));
  if (!abstract_tree)
  {
    getBehaviorContext()->logger->publishWarnMessage(name(), "Failed to deserialize octomap from planning scene.");
    return BT::NodeStatus::FAILURE;
  }

  auto* oc_tree = dynamic_cast<octomap::OcTree*>(abstract_tree.get());
  if (!oc_tree)
  {
    getBehaviorContext()->logger->publishWarnMessage(
        name(), "Octomap is not of type OcTree (id: " + std::string(abstract_tree->getTreeType()) + ").");
    return BT::NodeStatus::FAILURE;
  }

  const double resolution = oc_tree->getResolution();
  const int cells_radius = static_cast<int>(std::ceil(inflation_radius / resolution));

  // Collect all occupied leaf centers before modifying the tree
  std::vector<octomap::point3d> occupied_centers;
  for (auto it = oc_tree->begin_leafs(); it != oc_tree->end_leafs(); ++it)
  {
    if (oc_tree->isNodeOccupied(*it))
    {
      occupied_centers.push_back(it.getCoordinate());
    }
  }

  // For each occupied center, mark all voxels within inflation_radius as occupied
  for (const auto& center : occupied_centers)
  {
    for (int dx = -cells_radius; dx <= cells_radius; ++dx)
    {
      for (int dy = -cells_radius; dy <= cells_radius; ++dy)
      {
        for (int dz = -cells_radius; dz <= cells_radius; ++dz)
        {
          const double dist = std::sqrt(static_cast<double>(dx * dx + dy * dy + dz * dz)) * resolution;
          if (dist <= inflation_radius)
          {
            const octomap::point3d neighbor(static_cast<float>(center.x() + dx * resolution),
                                            static_cast<float>(center.y() + dy * resolution),
                                            static_cast<float>(center.z() + dz * resolution));
            oc_tree->updateNode(neighbor, true);
          }
        }
      }
    }
  }

  oc_tree->updateInnerOccupancy();

  // Serialize the inflated tree back to a message
  octomap_msgs::msg::Octomap inflated_octomap_msg;
  if (!octomap_msgs::binaryMapToMsg(*oc_tree, inflated_octomap_msg))
  {
    getBehaviorContext()->logger->publishWarnMessage(name(), "Failed to serialize inflated octomap to message.");
    return BT::NodeStatus::FAILURE;
  }
  inflated_octomap_msg.header = octomap_msg.header;

  // Build the output planning scene with the inflated octomap
  moveit_msgs::msg::PlanningScene output_scene;
  output_scene.world.octomap.octomap = inflated_octomap_msg;
  output_scene.world.octomap.header = scene.world.octomap.header;
  output_scene.world.octomap.origin = scene.world.octomap.origin;
  output_scene.is_diff = true;

  setOutput("inflated_scene", output_scene);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace planning_scene_behaviors
