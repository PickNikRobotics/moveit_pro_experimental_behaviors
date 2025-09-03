#pragma once

#include <experimental_behaviors/setup_mtc_debug_stage.hpp>
#include <moveit/task_constructor/stages.h>

namespace moveit_studio::behaviors {

class SetupMTCDebugStage final : public MTCStageHelper<SetupMTCDebugStage>
{
  MTC_STAGE_BEHAVIOR_INTERFACE(SetupMTCDebugStage, 
                               "My Custom Stage",
                               "to perform some custom MTC operation.")

public:
  static constexpr auto kPortIDCustomParam = "custom_param";

  static BT::PortsList providedPorts();

  tl::expected<std::unique_ptr<moveit::task_constructor::Stage>, std::string> 
  createStage(const StageInputs& inputs);
};

} // namespace moveit_studio::behaviors
