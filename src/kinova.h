#pragma once

#include <mc_rbdyn/RobotModule.h>
#include <mc_robots/api.h>

#include <optional>
#include <string>

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
    Screw,
    Hook
  };

  enum class Gripper
  {
    None,
    Robotiq2F85,
    Robotiq2F140
  };

  /** Configuration struct for constructing a KinovaRobotModule.
   *
   * Using a config struct avoids a long positional argument list and makes
   * call-sites self-documenting.  Every field has a sensible default so callers
   * only need to set what differs from the baseline "plain Kinova" variant.
   */
  struct Config
  {
    ForceSensor force_sensor{ForceSensor::None};
    EndEffector end_effector{EndEffector::None};
    Gripper gripper{Gripper::None};
    bool camera{false};
    bool callib{false};
    bool mujoco{false};
    bool canonical{false};
  };

  explicit KinovaRobotModule(const Config & config);
};

} // namespace mc_robots
