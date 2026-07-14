#include "kinova.h"

#include "config.h"

#include <RBDyn/Joint.h>
#include <RBDyn/MultiBody.h>
#include <RBDyn/parsers/urdf.h>

#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace mc_robots
{

// ─────────────────────────────────────────────────────────────────────────────
// Internal helpers — file-local, no linkage outside this TU
// ─────────────────────────────────────────────────────────────────────────────

namespace
{

// ── Gripper metadata ──────────────────────────────────────────────────────────

struct GripperSpec
{
  std::string actuatedJoint;
  std::vector<std::string> refJoints;
  std::vector<std::string> collisionLinks;
  std::vector<std::string> filteredLinks;
};

const GripperSpec & gripperSpec(KinovaRobotModule::Gripper gripper)
{
  static const GripperSpec robotiq2F85 = {
      "robotiq_85_left_knuckle_joint",
      {"robotiq_85_left_knuckle_joint", "robotiq_85_right_knuckle_joint", "robotiq_85_left_inner_knuckle_joint",
       "robotiq_85_right_inner_knuckle_joint", "robotiq_85_left_finger_tip_joint",
       "robotiq_85_right_finger_tip_joint"},
      {"robotiq_85_base_link", "robotiq_85_left_knuckle_link", "robotiq_85_right_knuckle_link",
       "robotiq_85_left_finger_link", "robotiq_85_right_finger_link", "robotiq_85_left_finger_tip_link",
       "robotiq_85_right_finger_tip_link"},
      {"robotiq_85_right_knuckle_link", "robotiq_85_right_finger_link", "robotiq_85_left_inner_knuckle_link",
       "robotiq_85_right_inner_knuckle_link", "robotiq_85_left_finger_tip_link",
       "robotiq_85_right_finger_tip_link"}};

  static const GripperSpec robotiq2F140 = {
      "finger_joint",
      {"finger_joint", "right_outer_knuckle_joint", "left_inner_knuckle_joint", "right_inner_knuckle_joint",
       "left_inner_finger_joint", "right_inner_finger_joint"},
      {"robotiq_140_base_link", "left_outer_knuckle", "right_outer_knuckle", "left_outer_finger",
       "right_outer_finger", "left_inner_knuckle", "right_inner_knuckle", "left_inner_finger",
       "right_inner_finger", "left_inner_finger_pad", "right_inner_finger_pad"},
      {"left_outer_knuckle", "left_outer_finger", "left_inner_finger", "left_inner_finger_pad",
       "left_inner_knuckle", "right_outer_knuckle", "right_outer_finger", "right_inner_finger",
       "right_inner_finger_pad", "right_inner_knuckle"}};

  switch(gripper)
  {
    case KinovaRobotModule::Gripper::Robotiq2F85:
      return robotiq2F85;
    case KinovaRobotModule::Gripper::Robotiq2F140:
      return robotiq2F140;
    case KinovaRobotModule::Gripper::None:
    default:
      throw std::invalid_argument("No GripperSpec defined for Gripper::None");
  }
}

// ── Predicate helpers ─────────────────────────────────────────────────────────

constexpr bool hasGripper(KinovaRobotModule::Gripper g) noexcept
{
  return g != KinovaRobotModule::Gripper::None;
}

constexpr bool hasBota(KinovaRobotModule::ForceSensor fs) noexcept
{
  return fs != KinovaRobotModule::ForceSensor::None;
}

constexpr bool supportsCallib(KinovaRobotModule::ForceSensor fs,
                              KinovaRobotModule::EndEffector ee) noexcept
{
  return hasBota(fs) && ee != KinovaRobotModule::EndEffector::None;
}

// ── Variant name resolution ───────────────────────────────────────────────────

/** Returns the URDF variant name that matches the given configuration.
 *
 * Validation of illegal combinations is done here once, rather than being
 * scattered across every branch of the old if-else chain.
 */
std::string resolveVariantName(const KinovaRobotModule::Config & cfg)
{
  using FS = KinovaRobotModule::ForceSensor;
  using EE = KinovaRobotModule::EndEffector;
  using GR = KinovaRobotModule::Gripper;

  // Gripper suffix shared by several BotaGenA variants
  auto gripperSuffix = [&]() -> std::string
  {
    switch(cfg.gripper)
    {
      case GR::None:
        return "";
      case GR::Robotiq2F85:
        return "_gripper";
      case GR::Robotiq2F140:
        return "_gripper_2f140";
    }
    throw std::invalid_argument("Unknown Gripper enum value");
  };

  // End-effector infix shared by several Bota variants
  auto eeSuffix = [&]() -> std::string
  {
    switch(cfg.end_effector)
    {
      case EE::None:
        return "";
      case EE::DS4:
        return "_ds4";
      case EE::Plate:
        return "_plate";
      case EE::Screw:
        return "_screw";
      case EE::Hook:
        return "_hook";
    }
    throw std::invalid_argument("Unknown EndEffector enum value");
  };

  // ── BotaGen0 ──────────────────────────────────────────────────────────────
  if(cfg.force_sensor == FS::BotaGen0)
  {
    if(hasGripper(cfg.gripper))
    {
      throw std::invalid_argument(
          "KinovaRobotModule: BotaGen0 variants do not support a gripper");
    }
    const std::string variant = "kinova_bota" + eeSuffix();
    mc_rtc::log::info("KinovaRobotModule uses the kinova variant: '{}'", variant);
    return variant;
  }

  // ── BotaGenA ─────────────────────────────────────────────────────────────
  if(cfg.force_sensor == FS::BotaGenA)
  {
    if(hasGripper(cfg.gripper) && cfg.end_effector != EE::None)
    {
      throw std::invalid_argument(
          "KinovaRobotModule: BotaGenA variants support either an end-effector "
          "or a gripper, not both");
    }
    const std::string variant = "kinova_bota_gena" + eeSuffix() + gripperSuffix();
    mc_rtc::log::info("KinovaRobotModule uses the kinova variant: '{}'", variant);
    return variant;
  }

  // ── No force sensor ───────────────────────────────────────────────────────
  const std::string cameraPrefix = cfg.camera ? "kinova_camera" : "kinova";

  if(cfg.gripper == GR::Robotiq2F85)
  {
    const std::string variant = cameraPrefix + "_gripper";
    mc_rtc::log::info("KinovaRobotModule uses the kinova variant: '{}'", variant);
    return variant;
  }
  if(cfg.gripper == GR::Robotiq2F140)
  {
    const std::string variant = cameraPrefix + "_gripper_2f140";
    mc_rtc::log::info("KinovaRobotModule uses the kinova variant: '{}'", variant);
    return variant;
  }

  // Plain kinova / kinova_camera
  mc_rtc::log::info("KinovaRobotModule uses the kinova variant: '{}'", cameraPrefix);
  return cameraPrefix;
}

/** Returns the canonical MuJoCo variant name when one is needed, or nullopt. */
std::optional<std::string> resolveCanonicalVariant(const KinovaRobotModule::Config & cfg)
{
  using FS = KinovaRobotModule::ForceSensor;
  using GR = KinovaRobotModule::Gripper;

  if(!cfg.mujoco || !hasGripper(cfg.gripper))
  {
    return std::nullopt;
  }

  if(cfg.force_sensor == FS::BotaGenA)
  {
    if(cfg.gripper == GR::Robotiq2F85)  { return "KinovaBotaGenARobotiq2F85MuJoCoCanonical"; }
    if(cfg.gripper == GR::Robotiq2F140) { return "KinovaBotaGenARobotiq2F140MuJoCoCanonical"; }
  }
  if(cfg.camera)
  {
    if(cfg.gripper == GR::Robotiq2F85)  { return "KinovaCameraRobotiq2F85MuJoCoCanonical"; }
    if(cfg.gripper == GR::Robotiq2F140) { return "KinovaCameraRobotiq2F140MuJoCoCanonical"; }
  }
  if(cfg.gripper == GR::Robotiq2F85)  { return "KinovaRobotiq2F85MuJoCoCanonical"; }
  if(cfg.gripper == GR::Robotiq2F140) { return "KinovaRobotiq2F140MuJoCoCanonical"; }

  return std::nullopt;
}

// ── RSDF directory resolution ─────────────────────────────────────────────────

fs::path resolveRsdfDir(const std::string & variant)
{
  const auto variantDir = fs::path(KINOVA_RSDF_DIR) / variant;
  if(fs::exists(variantDir))
  {
    return variantDir;
  }

  const auto defaultDir = fs::path(KINOVA_RSDF_DIR) / "kinova_default";
  if(fs::exists(defaultDir))
  {
    mc_rtc::log::warning(
        "No RSDF directory for variant '{}', falling back to '{}'", variant, defaultDir.string());
    return defaultDir;
  }

  return variantDir; // caller will deal with absence
}

// ── Joint limit helpers ───────────────────────────────────────────────────────

/** Applies position, velocity, and torque limits that differ from the URDF. */
void applyJointLimits(mc_rbdyn::RobotModule & mod, bool callib)
{
  auto setPositionLimits = [&](const std::string & name, double lo, double hi)
  {
    assert(hi > 0 && lo < 0);
    assert(mod._bounds[0].at(name).size() == 1);
    mod._bounds[0].at(name)[0] = lo;
    mod._bounds[1].at(name)[0] = hi;
  };

  auto setVelocityLimit = [&](const std::string & name, double limit)
  {
    assert(limit > 0);
    assert(mod._bounds[2].at(name).size() == 1);
    mod._bounds[2].at(name)[0] = -limit;
    mod._bounds[3].at(name)[0] = limit;
  };

  auto setTorqueLimit = [&](const std::string & name, double limit)
  {
    assert(limit > 0);
    assert(mod._bounds[4].at(name).size() == 1);
    mod._bounds[4].at(name)[0] = -limit;
    mod._bounds[5].at(name)[0] = limit;
  };

  // Position
  setPositionLimits("joint_2", -2.15, 2.15);
  setPositionLimits("joint_4", -2.45, callib ? 0.45 : 2.45);
  setPositionLimits("joint_6", -2.0, 2.0);
  if(callib)
  {
    setPositionLimits("joint_5", -3.14, 3.14);
    setPositionLimits("joint_7", -3.14, 3.14);
  }

  // Velocity  (joint_3 was duplicated in the original; joint_4 is now correct)
  setVelocityLimit("joint_1", 2.0944);
  setVelocityLimit("joint_2", 2.0944);
  setVelocityLimit("joint_3", 2.0944);
  setVelocityLimit("joint_4", 2.0944); // ← was missing in the original
  setVelocityLimit("joint_5", 3.049);
  setVelocityLimit("joint_6", 3.049);
  setVelocityLimit("joint_7", 3.049);

  // Torque
  setTorqueLimit("joint_1", 95);
  setTorqueLimit("joint_2", 95);
  setTorqueLimit("joint_3", 95);
  setTorqueLimit("joint_4", 95);
  setTorqueLimit("joint_5", 45);
  setTorqueLimit("joint_6", 45);
  setTorqueLimit("joint_7", 45);
}

// ── Drive-train parameters ────────────────────────────────────────────────────

void applyDrivetrainParameters(rbd::MultiBody & mb)
{
  // All seven joints share the same gear ratio
  constexpr double kGearRatio = 100.0;

  // Rotor inertia differs between the first four (larger) and last three joints
  constexpr double kInertiaLarge = 0.40e-4;
  constexpr double kInertiaSmall = 0.22e-4;

  const std::array<std::pair<std::string, double>, 7> joints = {{
      {"joint_1", kInertiaLarge},
      {"joint_2", kInertiaLarge},
      {"joint_3", kInertiaLarge},
      {"joint_4", kInertiaLarge},
      {"joint_5", kInertiaSmall},
      {"joint_6", kInertiaSmall},
      {"joint_7", kInertiaSmall},
  }};

  for(const auto & [name, inertia] : joints)
  {
    const int idx = mb.jointIndexByName(name);
    mb.setJointGearRatio(idx, kGearRatio);
    mb.setJointRotorInertia(idx, inertia);
  }
}

// ── Self-collision pairs ──────────────────────────────────────────────────────

/** Builds the minimal self-collision set for the given configuration. */
std::vector<mc_rbdyn::Collision>
buildSelfCollisions(const KinovaRobotModule::Config & cfg)
{
  using FS = KinovaRobotModule::ForceSensor;
  using EE = KinovaRobotModule::EndEffector;

  constexpr double i = 0.03, s = 0.015, d = 0.0;

  // Proximal links that must never touch the wrist/end region
  const std::vector<std::string> proximalLinks = {
      "base_link", "shoulder_link", "half_arm_1_link", "half_arm_2_link"};

  // Helper: add all proximal × {target} pairs
  std::vector<mc_rbdyn::Collision> cols;

  auto addPairs = [&](const std::string & target)
  {
    for(const auto & prox : proximalLinks)
    {
      cols.push_back({prox, target, i, s, d});
    }
  };

  // Wrist links always present
  for(const auto & wristLink :
      {"spherical_wrist_1_link", "spherical_wrist_2_link", "bracelet_link"})
  {
    addPairs(wristLink);
  }

  // Force-sensor-specific
  if(cfg.force_sensor == FS::BotaGen0)
  {
    addPairs("FT_adapter");
  }
  if(hasBota(cfg.force_sensor))
  {
    addPairs("FT_sensor_mounting");
  }

  // End-effector-specific
  const std::unordered_map<EE, std::string> eeLink = {
      {EE::DS4, "DS4_adapter"},
      {EE::Plate, "plate"},
      {EE::Screw, "screw"},
      {EE::Hook, "hook"},
  };
  if(auto it = eeLink.find(cfg.end_effector); it != eeLink.end())
  {
    addPairs(it->second);
  }

  // Gripper-specific
  if(hasGripper(cfg.gripper))
  {
    for(const auto & link : gripperSpec(cfg.gripper).collisionLinks)
    {
      addPairs(link);
    }
  }

  return cols;
}

// ── Gripper safety ────────────────────────────────────────────────────────────

mc_rbdyn::RobotModule::Gripper::Safety buildGripperSafety(const KinovaRobotModule::Config & cfg)
{
  using GR = KinovaRobotModule::Gripper;

  if(cfg.mujoco && (cfg.gripper == GR::Robotiq2F85 || cfg.gripper == GR::Robotiq2F140))
  {
    return {0.5, 0.18, 0.02, 10u};
  }
  if(cfg.mujoco)
  {
    return {0.5, 0.1, 0.05, 5u};
  }
  return {0.99, 0.05, 0.05, 1u};
}

} // anonymous namespace

// ─────────────────────────────────────────────────────────────────────────────
// KinovaRobotModule implementation
// ─────────────────────────────────────────────────────────────────────────────

KinovaRobotModule::KinovaRobotModule(const Config & cfg)
: mc_rbdyn::RobotModule(KINOVA_DESCRIPTION_PATH, resolveVariantName(cfg))
{
  // ── Validate configuration ────────────────────────────────────────────────

  if(cfg.callib && !supportsCallib(cfg.force_sensor, cfg.end_effector))
  {
    throw std::invalid_argument(
        "KinovaRobotModule: callib mode requires a Bota force sensor with a mounted end-effector");
  }

  mc_rtc::log::success("KinovaRobotModule loaded with name: {}", name);
  if(cfg.callib)
  {
    mc_rtc::log::info("KinovaRobotModule runs in callib mode for variant: '{}'", name);
  }

  // ── URDF paths ────────────────────────────────────────────────────────────

  urdf_path = fs::path(KINOVA_URDF_DIR) / (name + ".urdf");
  _real_urdf = urdf_path;

  // ── Canonical MuJoCo variant ──────────────────────────────────────────────

  if(cfg.mujoco && hasGripper(cfg.gripper) && !cfg.canonical)
  {
    if(auto canonical = resolveCanonicalVariant(cfg))
    {
      _canonicalParameters = {*canonical};
    }
  }

  // ── URDF parsing ──────────────────────────────────────────────────────────
  // MuJoCo non-canonical gripper variants keep filtered links in the model
  // (closed-chain mechanism) but do not remove them.

  if(cfg.mujoco && hasGripper(cfg.gripper) && !cfg.canonical)
  {
    init(rbd::parsers::from_urdf_file(urdf_path, rbd::parsers::ParserParameters{}
                                                     .fixed(true)
                                                     .filtered_links(gripperSpec(cfg.gripper).filteredLinks)
                                                     .remove_filtered_links(false)));
  }
  else
  {
    init(rbd::parsers::from_urdf_file(urdf_path, true));
  }

  // ── RSDF directory ────────────────────────────────────────────────────────

  rsdf_dir = resolveRsdfDir(name);
  mc_rtc::log::success("KinovaRobotModule using path \"{}\" for rsdf", rsdf_dir);

  // ── Reference joint order ─────────────────────────────────────────────────

  _ref_joint_order = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"};

  if(hasGripper(cfg.gripper))
  {
    _ref_joint_order.push_back(gripperSpec(cfg.gripper).actuatedJoint);
  }

  // ── Gripper module descriptor ─────────────────────────────────────────────

  if(hasGripper(cfg.gripper))
  {
    const auto & spec = gripperSpec(cfg.gripper);
    _grippers = {{"gripper", {spec.actuatedJoint}, true, buildGripperSafety(cfg)}};
  }

  // ── Joint limits ──────────────────────────────────────────────────────────

  applyJointLimits(*this, cfg.callib);

  // ── Drive-train parameters ────────────────────────────────────────────────

  applyDrivetrainParameters(mb);

  // ── Convex hulls ──────────────────────────────────────────────────────────

  const fs::path convexPath = fs::path(KINOVA_CONVEX_DIR) / "kinova";
  mc_rtc::log::success("KinovaRobotModule using path \"{}\" for convex", convexPath.string());

  for(const auto & body : mb.bodies())
  {
    const auto ch = convexPath / (body.name() + "-ch.txt");
    if(fs::exists(ch))
    {
      _convexHull[body.name()] = {body.name(), ch.string()};
    }
  }

  // ── Joint sensors (temperature / current logging) ─────────────────────────

  for(const auto & jointName : _ref_joint_order)
  {
    if(mb.jointIndexByName().count(jointName) != 0)
    {
      _jointSensors.emplace_back(jointName);
    }
  }

  // ── Force / body sensors ──────────────────────────────────────────────────

  if(hasBota(cfg.force_sensor))
  {
    _forceSensors.emplace_back("EEForceSensor", "FT_sensor_wrench", sva::PTransformd::Identity());
    _bodySensors.emplace_back("Accelerometer",  "FT_sensor_imu",    sva::PTransformd::Identity());
  }

  // ── Self-collision pairs ──────────────────────────────────────────────────

  _minimalSelfCollisions = buildSelfCollisions(cfg);
  _commonSelfCollisions  = _minimalSelfCollisions;

  // ── Default attitude and stance ───────────────────────────────────────────

  _default_attitude = {{1., 0., 0., 0., 0., 0., 0.}};

  _stance["joint_1"] = {0.0};
  _stance["joint_2"] = {0.2618};
  _stance["joint_3"] = {3.14};
  _stance["joint_4"] = {-2.269};
  _stance["joint_5"] = {0.0};
  _stance["joint_6"] = {0.959878729};
  _stance["joint_7"] = {1.57};

  mc_rtc::log::success("KinovaRobotModule uses urdf_path {}", urdf_path);
}

} // namespace mc_robots
