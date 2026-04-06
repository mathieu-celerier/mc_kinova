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

  enum class Gripper
  {
    None,
    Robotiq2F85,
    Robotiq2F140
  };

  KinovaRobotModule(bool callib,
                    ForceSensor force_sensor,
                    EndEffector end_effector = EndEffector::None,
                    bool camera = false,
                    Gripper gripper = Gripper::None,
                    bool mujoco = false,
                    bool canonical = false);
};

} // namespace mc_robots
