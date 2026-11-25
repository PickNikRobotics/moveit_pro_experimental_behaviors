#include "geometry_behaviors/express_world_vector_in_link_frame.hpp"

#include <vector>

#include <spdlog/spdlog.h>
#include <Eigen/Geometry>

namespace
{
constexpr auto kPortIDWorldVectorXYZ = "world_vector_xyz";
constexpr auto kPortIDLinkOrientationXYZW = "link_orientation_xyzw";
constexpr auto kPortIDLocalVectorXYZ = "local_vector_xyz";

inline constexpr auto kDescription = R"(
    <p>
        Re-expresses a 3D vector given in world frame as the equivalent vector in a link's
        local frame, using that link's current world orientation.
    </p>
    <p>
        Math: <code>local = q_link^{-1} * world</code>, where <code>q_link</code> is the
        link's orientation in world (e.g. the <code>orientation_xyzw</code> output of
        ExtractPoseOrientation applied to the link's TF). This is a pure rotation; no
        translation is involved.
    </p>
    <p>
        Primary use case: feeding a <i>world</i>-frame velocity intent (e.g. "descend in
        world -Z") to behaviors like CreateStampedTwist whose downstream consumer
        (velocity_force_controller via PublishVelocityForceCommand) interprets the twist's
        linear vector in the controller's own EE frame regardless of the TwistStamped's
        <code>header.frame_id</code>. Compute the per-cycle conversion here, then pass the
        result as <code>linear_velocity_xyz</code> to a CreateStampedTwist whose
        <code>reference_frame</code> matches the link.
    </p>
    <p>
        Both vectors are <code>std::vector&lt;double&gt;</code> of length 3 (x;y;z),
        matching CreateStampedTwist's port type. The orientation is xyzw.
    </p>
)";

}  // namespace

namespace geometry_behaviors
{

ExpressWorldVectorInLinkFrame::ExpressWorldVectorInLinkFrame(const std::string& name,
                                                             const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config)
{
}

BT::PortsList ExpressWorldVectorInLinkFrame::providedPorts()
{
  return {
    BT::InputPort<std::vector<double>>(kPortIDWorldVectorXYZ, "{world_vector_xyz}",
                                       "3D vector (x;y;z) expressed in world frame. The "
                                       "translation/length is preserved; only the rotation changes."),
    BT::InputPort<std::vector<double>>(kPortIDLinkOrientationXYZW, "{link_orientation_xyzw}",
                                       "Quaternion (x;y;z;w) representing the link's orientation in "
                                       "world. Must be the orientation of the same frame the result "
                                       "is being expressed in."),
    BT::OutputPort<std::vector<double>>(kPortIDLocalVectorXYZ, "{local_vector_xyz}",
                                        "3D vector (x;y;z) — the same physical vector as the input, "
                                        "but expressed in the link's local frame."),
  };
}

BT::KeyValueVector ExpressWorldVectorInLinkFrame::metadata()
{
  return { { "subcategory", "Math" }, { "description", kDescription } };
}

BT::NodeStatus ExpressWorldVectorInLinkFrame::tick()
{
  const auto world_vector_input = getInput<std::vector<double>>(kPortIDWorldVectorXYZ);
  if (!world_vector_input.has_value() || world_vector_input->size() != 3)
  {
    spdlog::error("ExpressWorldVectorInLinkFrame: 'world_vector_xyz' must be 3 values (x;y;z).");
    return BT::NodeStatus::FAILURE;
  }

  const auto link_orient_input = getInput<std::vector<double>>(kPortIDLinkOrientationXYZW);
  if (!link_orient_input.has_value() || link_orient_input->size() != 4)
  {
    spdlog::error("ExpressWorldVectorInLinkFrame: 'link_orientation_xyzw' must be 4 values (x;y;z;w).");
    return BT::NodeStatus::FAILURE;
  }

  const auto& w = world_vector_input.value();
  const auto& q_in = link_orient_input.value();

  const Eigen::Quaterniond q_link(q_in[3], q_in[0], q_in[1], q_in[2]);  // (w, x, y, z) constructor
  if (q_link.norm() < 1e-6)
  {
    spdlog::error("ExpressWorldVectorInLinkFrame: 'link_orientation_xyzw' has zero norm.");
    return BT::NodeStatus::FAILURE;
  }

  const Eigen::Vector3d v_world(w[0], w[1], w[2]);
  const Eigen::Vector3d v_local = q_link.normalized().inverse() * v_world;

  setOutput(kPortIDLocalVectorXYZ, std::vector<double>{ v_local.x(), v_local.y(), v_local.z() });

  spdlog::info("ExpressWorldVectorInLinkFrame: world=[{:.4f}, {:.4f}, {:.4f}] q=[{:.4f}, {:.4f}, {:.4f}, {:.4f}] -> "
               "local=[{:.4f}, {:.4f}, {:.4f}]",
               w[0], w[1], w[2], q_in[0], q_in[1], q_in[2], q_in[3], v_local.x(), v_local.y(), v_local.z());

  return BT::NodeStatus::SUCCESS;
}

}  // namespace geometry_behaviors
