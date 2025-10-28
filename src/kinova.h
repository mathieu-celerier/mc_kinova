#pragma once

#include <mc_rbdyn/RobotModule.h>

#include <mc_robots/api.h>

namespace mc_robots
{

struct MC_ROBOTS_DLLAPI KinovaRobotModule : public mc_rbdyn::RobotModule
{
  KinovaRobotModule(bool callib, bool use_bota, bool use_ds4 = false);
};

} // namespace mc_robots
