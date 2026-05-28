#pragma once

#include <mc_rbdyn/RobotModule.h>

#include <mc_robots/api.h>

namespace mc_robots
{

struct MC_ROBOTS_DLLAPI KinovaRobotModule : public mc_rbdyn::RobotModule
{
  enum class EndEffector
  {
    None,
    DS4,
    Plate,
    Screw
  };

  KinovaRobotModule(const std::string & name, bool callib, bool fixed = true);
};

} // namespace mc_robots
