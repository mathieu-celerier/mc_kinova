#pragma once

#include <mc_rbdyn/RobotModule.h>

#include <mc_robots/api.h>

namespace mc_robots
{

struct MC_ROBOTS_DLLAPI KinovaRobotModule : public mc_rbdyn::RobotModule
{
  enum class ForceSensor
  {
    None,
    BotaGen0,
    BotaGenA
  };

  enum class EndEffector
  {
    None,
    DS4,
    Plate,
    Screw
  };

  KinovaRobotModule(bool callib,
                    ForceSensor force_sensor,
                    EndEffector end_effector = EndEffector::None,
                    bool camera = false,
                    bool gripper = false);
};

} // namespace mc_robots
