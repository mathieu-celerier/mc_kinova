#include "kinova.h"

#include <mc_rbdyn/RobotModuleMacros.h>
#include <mc_rtc/logging.h>

#include <functional>
#include <string>
#include <unordered_map>

// ─────────────────────────────────────────────────────────────────────────────
// Registry
//
// Each entry maps a public robot name to a factory lambda.  Adding a new
// variant only requires adding one line here — the MC_RTC_ROBOT_MODULE export
// is derived automatically from the registry keys, so both stay in sync.
// ─────────────────────────────────────────────────────────────────────────────

namespace
{

using Config  = mc_robots::KinovaRobotModule::Config;
using FS      = mc_robots::KinovaRobotModule::ForceSensor;
using EE      = mc_robots::KinovaRobotModule::EndEffector;
using GR      = mc_robots::KinovaRobotModule::Gripper;
using Factory = std::function<mc_rbdyn::RobotModule *()>;

// clang-format off
const std::unordered_map<std::string, Factory> & registry()
{
  static const std::unordered_map<std::string, Factory> reg = {

    // ── Plain Kinova ────────────────────────────────────────────────────────
    {"Kinova",
        [] { return new mc_robots::KinovaRobotModule(Config{}); }},

    // ── BotaGen0 (no gripper support) ───────────────────────────────────────
    {"KinovaBota",
        [] { return new mc_robots::KinovaRobotModule(Config{.force_sensor = FS::BotaGen0}); }},
    {"KinovaBotaDS4",
        [] { return new mc_robots::KinovaRobotModule(Config{.force_sensor = FS::BotaGen0, .end_effector = EE::DS4}); }},
    {"KinovaBotaDS4Callib",
        [] { return new mc_robots::KinovaRobotModule(Config{.force_sensor = FS::BotaGen0, .end_effector = EE::DS4, .callib = true}); }},
    {"KinovaBotaPlate",
        [] { return new mc_robots::KinovaRobotModule(Config{.force_sensor = FS::BotaGen0, .end_effector = EE::Plate}); }},
    {"KinovaBotaPlateCallib",
        [] { return new mc_robots::KinovaRobotModule(Config{.force_sensor = FS::BotaGen0, .end_effector = EE::Plate, .callib = true}); }},
    {"KinovaBotaScrew",
        [] { return new mc_robots::KinovaRobotModule(Config{.force_sensor = FS::BotaGen0, .end_effector = EE::Screw}); }},
    {"KinovaBotaScrewCallib",
        [] { return new mc_robots::KinovaRobotModule(Config{.force_sensor = FS::BotaGen0, .end_effector = EE::Screw, .callib = true}); }},
    {"KinovaBotaHook",
        [] { return new mc_robots::KinovaRobotModule(Config{.force_sensor = FS::BotaGen0, .end_effector = EE::Hook}); }},
    {"KinovaBotaHookCallib",
        [] { return new mc_robots::KinovaRobotModule(Config{.force_sensor = FS::BotaGen0, .end_effector = EE::Hook, .callib = true}); }},

    // ── BotaGenA ────────────────────────────────────────────────────────────
    {"KinovaBotaGenA",
        [] { return new mc_robots::KinovaRobotModule(Config{.force_sensor = FS::BotaGenA}); }},
    {"KinovaBotaGenADS4",
        [] { return new mc_robots::KinovaRobotModule(Config{.force_sensor = FS::BotaGenA, .end_effector = EE::DS4}); }},
    {"KinovaBotaGenADS4Callib",
        [] { return new mc_robots::KinovaRobotModule(Config{.force_sensor = FS::BotaGenA, .end_effector = EE::DS4, .callib = true}); }},
    {"KinovaBotaGenAPlate",
        [] { return new mc_robots::KinovaRobotModule(Config{.force_sensor = FS::BotaGenA, .end_effector = EE::Plate}); }},
    {"KinovaBotaGenAPlateCallib",
        [] { return new mc_robots::KinovaRobotModule(Config{.force_sensor = FS::BotaGenA, .end_effector = EE::Plate, .callib = true}); }},
    {"KinovaBotaGenAScrew",
        [] { return new mc_robots::KinovaRobotModule(Config{.force_sensor = FS::BotaGenA, .end_effector = EE::Screw}); }},
    {"KinovaBotaGenAScrewCallib",
        [] { return new mc_robots::KinovaRobotModule(Config{.force_sensor = FS::BotaGenA, .end_effector = EE::Screw, .callib = true}); }},
    {"KinovaBotaGenAHook",
        [] { return new mc_robots::KinovaRobotModule(Config{.force_sensor = FS::BotaGenA, .end_effector = EE::Hook}); }},
    {"KinovaBotaGenAHookCallib",
        [] { return new mc_robots::KinovaRobotModule(Config{.force_sensor = FS::BotaGenA, .end_effector = EE::Hook, .callib = true}); }},

    // ── BotaGenA + grippers ─────────────────────────────────────────────────
    // "KinovaBotaGenAGripper" is kept as a documented alias for Robotiq2F85
    {"KinovaBotaGenAGripper",
        [] { return new mc_robots::KinovaRobotModule(Config{.force_sensor = FS::BotaGenA, .gripper = GR::Robotiq2F85}); }},
    {"KinovaBotaGenARobotiq2F85",
        [] { return new mc_robots::KinovaRobotModule(Config{.force_sensor = FS::BotaGenA, .gripper = GR::Robotiq2F85}); }},
    {"KinovaBotaGenARobotiq2F140",
        [] { return new mc_robots::KinovaRobotModule(Config{.force_sensor = FS::BotaGenA, .gripper = GR::Robotiq2F140}); }},

    // ── BotaGenA + grippers + MuJoCo ────────────────────────────────────────
    {"KinovaBotaGenAGripperMuJoCo",
        [] { return new mc_robots::KinovaRobotModule(Config{.force_sensor = FS::BotaGenA, .gripper = GR::Robotiq2F85, .mujoco = true}); }},
    {"KinovaBotaGenARobotiq2F85MuJoCo",
        [] { return new mc_robots::KinovaRobotModule(Config{.force_sensor = FS::BotaGenA, .gripper = GR::Robotiq2F85, .mujoco = true}); }},
    {"KinovaBotaGenARobotiq2F85MuJoCoCanonical",
        [] { return new mc_robots::KinovaRobotModule(Config{.force_sensor = FS::BotaGenA, .gripper = GR::Robotiq2F85, .mujoco = true, .canonical = true}); }},
    {"KinovaBotaGenARobotiq2F140MuJoCo",
        [] { return new mc_robots::KinovaRobotModule(Config{.force_sensor = FS::BotaGenA, .gripper = GR::Robotiq2F140, .mujoco = true}); }},
    {"KinovaBotaGenARobotiq2F140MuJoCoCanonical",
        [] { return new mc_robots::KinovaRobotModule(Config{.force_sensor = FS::BotaGenA, .gripper = GR::Robotiq2F140, .mujoco = true, .canonical = true}); }},

    // ── Camera variants ─────────────────────────────────────────────────────
    {"KinovaCamera",
        [] { return new mc_robots::KinovaRobotModule(Config{.camera = true}); }},

    // ── Camera + grippers ───────────────────────────────────────────────────
    // "KinovaCameraGripper" is kept as a documented alias for Robotiq2F85
    {"KinovaCameraGripper",
        [] { return new mc_robots::KinovaRobotModule(Config{.gripper = GR::Robotiq2F85, .camera = true}); }},
    {"KinovaCameraRobotiq2F85",
        [] { return new mc_robots::KinovaRobotModule(Config{.gripper = GR::Robotiq2F85, .camera = true}); }},
    {"KinovaCameraRobotiq2F140",
        [] { return new mc_robots::KinovaRobotModule(Config{.gripper = GR::Robotiq2F140, .camera = true}); }},

    // ── Camera + grippers + MuJoCo ──────────────────────────────────────────
    {"KinovaCameraGripperMuJoCo",
        [] { return new mc_robots::KinovaRobotModule(Config{.gripper = GR::Robotiq2F85, .camera = true, .mujoco = true}); }},
    {"KinovaCameraRobotiq2F85MuJoCo",
        [] { return new mc_robots::KinovaRobotModule(Config{.gripper = GR::Robotiq2F85, .camera = true, .mujoco = true}); }},
    {"KinovaCameraRobotiq2F85MuJoCoCanonical",
        [] { return new mc_robots::KinovaRobotModule(Config{.gripper = GR::Robotiq2F85, .camera = true, .mujoco = true, .canonical = true}); }},
    {"KinovaCameraRobotiq2F140MuJoCo",
        [] { return new mc_robots::KinovaRobotModule(Config{.gripper = GR::Robotiq2F140, .camera = true, .mujoco = true}); }},
    {"KinovaCameraRobotiq2F140MuJoCoCanonical",
        [] { return new mc_robots::KinovaRobotModule(Config{.gripper = GR::Robotiq2F140, .camera = true, .mujoco = true, .canonical = true}); }},

    // ── Plain grippers ──────────────────────────────────────────────────────
    // "KinovaGripper" is kept as a documented alias for Robotiq2F85
    {"KinovaGripper",
        [] { return new mc_robots::KinovaRobotModule(Config{.gripper = GR::Robotiq2F85}); }},
    {"KinovaRobotiq2F85",
        [] { return new mc_robots::KinovaRobotModule(Config{.gripper = GR::Robotiq2F85}); }},
    {"KinovaRobotiq2F140",
        [] { return new mc_robots::KinovaRobotModule(Config{.gripper = GR::Robotiq2F140}); }},

    // ── Plain grippers + MuJoCo ─────────────────────────────────────────────
    {"KinovaGripperMuJoCo",
        [] { return new mc_robots::KinovaRobotModule(Config{.gripper = GR::Robotiq2F85, .mujoco = true}); }},
    {"KinovaRobotiq2F85MuJoCo",
        [] { return new mc_robots::KinovaRobotModule(Config{.gripper = GR::Robotiq2F85, .mujoco = true}); }},
    {"KinovaRobotiq2F85MuJoCoCanonical",
        [] { return new mc_robots::KinovaRobotModule(Config{.gripper = GR::Robotiq2F85, .mujoco = true, .canonical = true}); }},
    {"KinovaRobotiq2F140MuJoCo",
        [] { return new mc_robots::KinovaRobotModule(Config{.gripper = GR::Robotiq2F140, .mujoco = true}); }},
    {"KinovaRobotiq2F140MuJoCoCanonical",
        [] { return new mc_robots::KinovaRobotModule(Config{.gripper = GR::Robotiq2F140, .mujoco = true, .canonical = true}); }},
  };
  // clang-format on
  return reg;
}

} // anonymous namespace

// ─────────────────────────────────────────────────────────────────────────────
// mc_rtc plugin entry points
// ─────────────────────────────────────────────────────────────────────────────

extern "C"
{
  ROBOT_MODULE_API void MC_RTC_ROBOT_MODULE(std::vector<std::string> & names)
  {
    names.clear();
    names.reserve(registry().size());
    for(const auto & [name, _] : registry())
    {
      names.push_back(name);
    }
  }

  ROBOT_MODULE_API void destroy(mc_rbdyn::RobotModule * ptr)
  {
    delete ptr;
  }

  ROBOT_MODULE_API mc_rbdyn::RobotModule * create(const std::string & name)
  {
    ROBOT_MODULE_CHECK_VERSION("Kinova")

    const auto & reg = registry();
    const auto it = reg.find(name);
    if(it == reg.end())
    {
      mc_rtc::log::error("KinovaRobotModule: unknown variant '{}'", name);
      return nullptr;
    }
    return it->second();
  }
}
