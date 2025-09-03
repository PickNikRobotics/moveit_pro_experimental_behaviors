// Copyright 2025 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <moveit/robot_model/robot_model.hpp>


#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stage.h>

#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node.hpp>
#include <moveit_studio_behavior_interface/get_required_ports.hpp>
#include <moveit_studio_behavior_interface/metadata_fields.hpp>

#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <type_traits>
#include <tl_expected/expected.hpp>

namespace moveit_studio::behaviors
{

/**
 * @brief Common port IDs used across MTC behaviors
 */
struct MTCPortIDs
{
  static constexpr auto kTask = "task";
  static constexpr auto kPlanningGroupName = "planning_group_name";  
  static constexpr auto kVelocityScale = "velocity_scale";
  static constexpr auto kAccelerationScale = "acceleration_scale";
  static constexpr auto kIgnoreEnvironmentCollisions = "ignore_environment_collisions";
};

/**
 * @brief Common port definitions for MTC behaviors
 */
class MTCCommonPorts
{
public:
  /**
   * @brief Get the standard MTC task port
   */
  static BT::PortsList taskPort()
  {
    return {
      BT::BidirectionalPort<moveit::task_constructor::TaskPtr>(MTCPortIDs::kTask, "{mtc_task}", "MoveIt Task Constructor task.")
    };
  }

  /**
   * @brief Get standard planning group port
   */
  static BT::PortsList planningGroupPort()
  {
    return {
      BT::InputPort<std::string>(MTCPortIDs::kPlanningGroupName, "manipulator", 
                                 "Name of the MoveIt planning group which will perform the motion.")
    };
  }

  /**
   * @brief Get standard scaling ports (velocity and acceleration)
   */
  static BT::PortsList scalingPorts()
  {
    return {
      BT::InputPort<double>(MTCPortIDs::kVelocityScale, 1.0,
                            "Scale the maximum velocity of the trajectory by this factor relative to the robot's joint "
                            "limits. Must be greater than 0.0 and less than or equal to 1.0."),
      BT::InputPort<double>(MTCPortIDs::kAccelerationScale, 1.0,
                            "Scale the maximum acceleration of the trajectory by this factor relative to the robot's "
                            "joint limits. Must be greater than 0.0 and less than or equal to 1.0.")
    };
  }

  /**
   * @brief Get environment collision ignore port
   */
  static BT::PortsList environmentCollisionPort()
  {
    return {
      BT::InputPort<bool>(MTCPortIDs::kIgnoreEnvironmentCollisions, false,
                          "If true, ignore collisions with the environment during planning. Self-collisions won't be ignored.")
    };
  }

  /**
   * @brief Get all common MTC ports (task + planning group + scaling + environment collision)
   */
  static BT::PortsList commonPorts()
  {
    auto ports = taskPort();
    auto group_ports = planningGroupPort();
    auto scaling_ports = scalingPorts();
    auto collision_ports = environmentCollisionPort();

    for (const auto& port : group_ports) ports.insert(port);
    for (const auto& port : scaling_ports) ports.insert(port);
    for (const auto& port : collision_ports) ports.insert(port);

    return ports;
  }

  /**
   * @brief Merge additional ports with common ports
   */
  static BT::PortsList mergePorts(const BT::PortsList& additional_ports)
  {
    auto ports = commonPorts();
    for (const auto& port : additional_ports) ports.insert(port);
    return ports;
  }
};

/**
 * @brief Standard metadata for MTC behaviors
 */
class MTCMetadata
{
public:
  /**
   * @brief Create standard MTC metadata with custom description
   */
  static BT::KeyValueVector create(const std::string& description, 
                                   const std::string& subcategory = "Motion - Task Planning")
  {
    return {
      { kSubcategoryMetadataKey, subcategory },
      { kDescriptionMetadataKey, description }
    };
  }
};

/**
 * @brief Common validation utilities for MTC behaviors
 */
class MTCValidation
{
public:
  /**
   * @brief Validate velocity scaling factor
   */
  static tl::expected<void, std::string> validateVelocityScale(double velocity_scale)
  {
    if (velocity_scale < std::numeric_limits<double>::epsilon() || velocity_scale > 1.0)
    {
      return tl::make_unexpected("The provided velocity scale factor of " + std::to_string(velocity_scale) +
                                 " is not within the range (0.0, 1.0].");
    }
    return {};
  }

  /**
   * @brief Validate acceleration scaling factor
   */
  static tl::expected<void, std::string> validateAccelerationScale(double acceleration_scale)
  {
    if (acceleration_scale < std::numeric_limits<double>::epsilon() || acceleration_scale > 1.0)
    {
      return tl::make_unexpected("The provided acceleration scale factor of " + std::to_string(acceleration_scale) +
                                 " is not within the range (0.0, 1.0].");
    }
    return {};
  }

  /**
   * @brief Validate both scaling factors
   */
  static tl::expected<void, std::string> validateScaling(double velocity_scale, double acceleration_scale)
  {
    if (auto result = validateVelocityScale(velocity_scale); !result)
    {
      return result;
    }
    return validateAccelerationScale(acceleration_scale);
  }

  /**
   * @brief Validate that a frame exists in the robot model
   */
  static tl::expected<void, std::string> validateFrame(const std::shared_ptr<const moveit::core::RobotModel>& robot_model,
                                                       const std::string& frame_name, const std::string& frame_description = "Frame")
  {
    if (robot_model->getLinkModel(frame_name) == nullptr)
    {
      return tl::make_unexpected(frame_description + " `" + frame_name + "` is not a robot link.");
    }
    return {};
  }

  /**
   * @brief Validate that a planning group exists in the robot model
   */
  static tl::expected<void, std::string> validatePlanningGroup(const std::shared_ptr<const moveit::core::RobotModel>& robot_model,
                                                               const std::string& planning_group_name)
  {
    if (robot_model->getJointModelGroup(planning_group_name) == nullptr)
    {
      const auto& known_group_names = robot_model->getJointModelGroupNames();
      std::string group_msg = "[";
      for (size_t i = 0; i < known_group_names.size(); ++i)
      {
        group_msg.append(known_group_names[i]);
        if (i + 1 < known_group_names.size())
          group_msg.append(", ");
      }
      group_msg.append("]");

      return tl::make_unexpected("Group `" + planning_group_name +
                                 "` is not a robot joint model group name. Known group names are: " + group_msg);
    }
    return {};
  }
};

/**
 * @brief Template helper for creating MTC stage behaviors with common functionality
 * 
 * This class provides a template for MTC behaviors that follow the standard pattern:
 * 1. Extract common ports (task, planning group, scaling factors)
 * 2. Validate common inputs
 * 3. Create and configure an MTC stage using a provided factory function
 * 4. Add the stage to the task
 *
 * Usage example:
 * 
 * class SetupMTCMyBehavior : public MTCStageHelper<SetupMTCMyBehavior>
 * {
 * public:
 *   static constexpr auto kDefaultStageName = "My Stage";
 *   
 *   SetupMTCMyBehavior(const std::string& name, const BT::NodeConfiguration& config,
 *                      const std::shared_ptr<BehaviorContext>& shared_resources)
 *     : MTCStageHelper(name, config, shared_resources) {}
 *
 *   static BT::PortsList providedPorts() {
 *     return MTCCommonPorts::mergePorts({
 *       BT::InputPort<double>("my_param", 1.0, "My custom parameter")
 *     });
 *   }
 *
 *   static BT::KeyValueVector metadata() {
 *     return MTCMetadata::create("Description of my behavior");
 *   }
 *
 *   auto createStage(const StageInputs& inputs) {
 *     // Custom stage creation logic
 *     auto stage = std::make_unique<moveit::task_constructor::stages::SomeStage>(kDefaultStageName);
 *     // Configure stage using inputs...
 *     return stage;
 *   }
 * };
 */
template <typename DerivedT>
class MTCStageHelper : public SharedResourcesNode<BT::SyncActionNode>
{
public:
  /**
   * @brief Structure containing common MTC inputs extracted from ports
   */
  struct StageInputs
  {
    moveit::task_constructor::TaskPtr task;
    std::string planning_group_name;
    double velocity_scale;
    double acceleration_scale;
    bool ignore_environment_collisions;
  };

  MTCStageHelper(const std::string& name, const BT::NodeConfiguration& config,
                 const std::shared_ptr<BehaviorContext>& shared_resources)
    : SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
  {
  }

  /**
   * @brief Standard tick implementation that handles common MTC behavior pattern
   * 
   * This method:
   * 1. Extracts common inputs from ports
   * 2. Validates scaling factors
   * 3. Calls derived class's createStage() method
   * 4. Adds the stage to the task
   * 
   * The derived class must implement createStage() method that returns a unique_ptr to an MTC stage.
   */
  BT::NodeStatus tick() override
  {
    // Extract common inputs
    auto stage_inputs_result = extractCommonInputs();
    if (!stage_inputs_result.has_value())
    {
      shared_resources_->logger->publishFailureMessage(this->name(), stage_inputs_result.error());
      return BT::NodeStatus::FAILURE;
    }

    const auto& stage_inputs = stage_inputs_result.value();

    // Validate common inputs
    if (auto validation_result = MTCValidation::validateScaling(stage_inputs.velocity_scale, stage_inputs.acceleration_scale);
        !validation_result)
    {
      shared_resources_->logger->publishFailureMessage(this->name(), validation_result.error());
      return BT::NodeStatus::FAILURE;
    }

    // Call derived class to create the stage
    auto stage_result = static_cast<DerivedT*>(this)->createStage(stage_inputs);
    if (!stage_result.has_value())
    {
      shared_resources_->logger->publishFailureMessage(this->name(), stage_result.error());
      return BT::NodeStatus::FAILURE;
    }

    auto stage = std::move(stage_result.value());

    // Add stage to task
    stage_inputs.task->add(std::move(stage));

    return BT::NodeStatus::SUCCESS;
  }

protected:
  /**
   * @brief Extract common inputs from behavior ports
   * 
   * Derived classes can override this if they need different common inputs,
   * or they can use the protected getStageInputs() method for custom extraction.
   */
  virtual tl::expected<StageInputs, std::string> extractCommonInputs()
  {
    const auto ports = getRequiredInputs(
        this->template getInput<moveit::task_constructor::TaskPtr>(MTCPortIDs::kTask),
        this->template getInput<std::string>(MTCPortIDs::kPlanningGroupName),
        this->template getInput<double>(MTCPortIDs::kVelocityScale),
        this->template getInput<double>(MTCPortIDs::kAccelerationScale),
        this->template getInput<bool>(MTCPortIDs::kIgnoreEnvironmentCollisions)
    );

    if (!ports.has_value())
    {
      return tl::make_unexpected("Failed to get required input: " + ports.error());
    }

    const auto& [task, planning_group_name, velocity_scale, acceleration_scale, ignore_environment_collisions] = ports.value();

    return StageInputs{
      .task = task,
      .planning_group_name = planning_group_name,
      .velocity_scale = velocity_scale,
      .acceleration_scale = acceleration_scale,
      .ignore_environment_collisions = ignore_environment_collisions
    };
  }

  /**
   * @brief Helper to get stage inputs for use in derived class implementations
   */
  tl::expected<StageInputs, std::string> getStageInputs() { 
    return extractCommonInputs(); 
  }

private:
  // No member variables needed - inputs are extracted on demand
};

/**
 * @brief Convenience macro to define required methods for MTC stage behaviors
 * 
 * Usage:
 * MTC_STAGE_BEHAVIOR_INTERFACE(MyBehavior, "My Stage Name", "Description of behavior")
 */
#define MTC_STAGE_BEHAVIOR_INTERFACE(ClassName, StageName, Description) \
public: \
  static constexpr auto kDefaultStageName = StageName; \
  \
  ClassName(const std::string& name, const BT::NodeConfiguration& config, \
            const std::shared_ptr<BehaviorContext>& shared_resources) \
    : MTCStageHelper<ClassName>(name, config, shared_resources) {} \
  \
  static BT::KeyValueVector metadata() \
  { \
    return MTCMetadata::create(std::string("<p>Given an existing MTC Task object, append a MTC stage, named '") + \
                               StageName + "', " + Description + "</p>"); \
  }

}  // namespace moveit_studio::behaviors
