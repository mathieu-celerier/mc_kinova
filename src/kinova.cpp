#include "kinova.h"

#include "config.h"

#include <RBDyn/Joint.h>
#include <RBDyn/MultiBody.h>
#include <RBDyn/parsers/urdf.h>

#include <filesystem>
#include <stdexcept>
namespace fs = std::filesystem;

namespace mc_robots
{

inline static bool hasBota(KinovaRobotModule::ForceSensor force_sensor)
{
  return force_sensor != KinovaRobotModule::ForceSensor::None;
}

inline static bool supportsCallib(KinovaRobotModule::ForceSensor force_sensor,
                                  KinovaRobotModule::EndEffector end_effector)
{
  return hasBota(force_sensor) && end_effector != KinovaRobotModule::EndEffector::None;
}

struct GripperSpec
{
  std::string actuatedJoint;
  std::vector<std::string> refJoints;
  std::vector<std::string> collisionLinks;
};

inline static bool hasGripper(KinovaRobotModule::Gripper gripper)
{
  return gripper != KinovaRobotModule::Gripper::None;
}

inline static std::string gripperVariantSuffix(KinovaRobotModule::Gripper gripper)
{
  switch(gripper)
  {
    case KinovaRobotModule::Gripper::None:
      return "";
    case KinovaRobotModule::Gripper::Robotiq2F85:
      return "_gripper";
    case KinovaRobotModule::Gripper::Robotiq2F140:
      return "_gripper_2f140";
  }
  throw std::invalid_argument("Unsupported gripper variant");
}

inline static const GripperSpec & gripperSpec(KinovaRobotModule::Gripper gripper)
{
  static const GripperSpec robotiq2F85 = {
      "robotiq_85_left_knuckle_joint",
      {"robotiq_85_left_knuckle_joint", "robotiq_85_right_knuckle_joint", "robotiq_85_left_inner_knuckle_joint",
       "robotiq_85_right_inner_knuckle_joint", "robotiq_85_left_finger_tip_joint", "robotiq_85_right_finger_tip_joint"},
      {"robotiq_85_base_link", "robotiq_85_left_knuckle_link", "robotiq_85_right_knuckle_link",
       "robotiq_85_left_finger_link", "robotiq_85_right_finger_link", "robotiq_85_left_finger_tip_link",
       "robotiq_85_right_finger_tip_link"}};
  static const GripperSpec robotiq2F140 = {
      "finger_joint",
      {"finger_joint", "right_outer_knuckle_joint", "left_inner_knuckle_joint", "right_inner_knuckle_joint",
       "left_inner_finger_joint", "right_inner_finger_joint"},
      {"robotiq_140_base_link", "left_outer_knuckle", "right_outer_knuckle", "left_outer_finger", "right_outer_finger",
       "left_inner_knuckle", "right_inner_knuckle", "left_inner_finger", "right_inner_finger", "left_inner_finger_pad",
       "right_inner_finger_pad"}};

  switch(gripper)
  {
    case KinovaRobotModule::Gripper::Robotiq2F85:
      return robotiq2F85;
    case KinovaRobotModule::Gripper::Robotiq2F140:
      return robotiq2F140;
    case KinovaRobotModule::Gripper::None:
    default:
      throw std::invalid_argument("No gripper metadata is defined for the requested gripper variant");
  }
}

inline static std::string kinovaVariant(
    KinovaRobotModule::ForceSensor force_sensor,
    KinovaRobotModule::EndEffector end_effector = KinovaRobotModule::EndEffector::None,
    bool camera = false,
    KinovaRobotModule::Gripper gripper = KinovaRobotModule::Gripper::None)
{
  if(force_sensor == KinovaRobotModule::ForceSensor::BotaGen0)
  {
    if(hasGripper(gripper))
    {
      throw std::invalid_argument("KinovaRobotModule does not provide generated Bota Gen0 variants with a gripper");
    }
    if(end_effector == KinovaRobotModule::EndEffector::DS4)
    {
      mc_rtc::log::info("KinovaRobotModule uses the kinova variant: 'kinova_bota_ds4'");
      return "kinova_bota_ds4";
    }
    else if(end_effector == KinovaRobotModule::EndEffector::Plate)
    {
      mc_rtc::log::info("KinovaRobotModule uses the kinova variant: 'kinova_bota_plate'");
      return "kinova_bota_plate";
    }
    else if(end_effector == KinovaRobotModule::EndEffector::Screw)
    {
      mc_rtc::log::info("KinovaRobotModule uses the kinova variant: 'kinova_bota_screw'");
      return "kinova_bota_screw";
    }
    mc_rtc::log::info("KinovaRobotModule uses the kinova variant: 'kinova_bota'");
    return "kinova_bota";
  }
  if(force_sensor == KinovaRobotModule::ForceSensor::BotaGenA)
  {
    if(hasGripper(gripper) && end_effector != KinovaRobotModule::EndEffector::None)
    {
      throw std::invalid_argument(
          "KinovaRobotModule Bota GenA variants support either an end effector or a gripper, not both");
    }
    const auto gripperSuffix = gripperVariantSuffix(gripper);
    if(end_effector == KinovaRobotModule::EndEffector::DS4)
    {
      mc_rtc::log::info("KinovaRobotModule uses the kinova variant: 'kinova_bota_gena_ds4{}'", gripperSuffix);
      return "kinova_bota_gena_ds4" + gripperSuffix;
    }
    else if(end_effector == KinovaRobotModule::EndEffector::Plate)
    {
      mc_rtc::log::info("KinovaRobotModule uses the kinova variant: 'kinova_bota_gena_plate{}'", gripperSuffix);
      return "kinova_bota_gena_plate" + gripperSuffix;
    }
    else if(end_effector == KinovaRobotModule::EndEffector::Screw)
    {
      mc_rtc::log::info("KinovaRobotModule uses the kinova variant: 'kinova_bota_gena_screw{}'", gripperSuffix);
      return "kinova_bota_gena_screw" + gripperSuffix;
    }
    mc_rtc::log::info("KinovaRobotModule uses the kinova variant: 'kinova_bota_gena{}'", gripperSuffix);
    return "kinova_bota_gena" + gripperSuffix;
  }
  if(camera)
  {
    if(gripper == KinovaRobotModule::Gripper::Robotiq2F85)
    {
      mc_rtc::log::info("KinovaRobotModule uses the kinova variant: 'kinova_camera_gripper'");
      return "kinova_camera_gripper";
    }
    if(gripper == KinovaRobotModule::Gripper::Robotiq2F140)
    {
      mc_rtc::log::info("KinovaRobotModule uses the kinova variant: 'kinova_camera_gripper_2f140'");
      return "kinova_camera_gripper_2f140";
    }
    mc_rtc::log::info("KinovaRobotModule uses the kinova variant: 'kinova_camera'");
    return "kinova_camera";
  }
  if(gripper == KinovaRobotModule::Gripper::Robotiq2F85)
  {
    mc_rtc::log::info("KinovaRobotModule uses the kinova variant: 'kinova_gripper'");
    return "kinova_gripper";
  }
  if(gripper == KinovaRobotModule::Gripper::Robotiq2F140)
  {
    mc_rtc::log::info("KinovaRobotModule uses the kinova variant: 'kinova_gripper_2f140'");
    return "kinova_gripper_2f140";
  }
  mc_rtc::log::info("KinovaRobotModule uses the kinova variant: 'kinova'");
  return "kinova";
}

inline static fs::path kinovaRsdfDir(const std::string & variant)
{
  const auto variantDir = fs::path(KINOVA_RSDF_DIR) / variant;
  if(fs::exists(variantDir))
  {
    return variantDir;
  }

  const auto defaultDir = fs::path(KINOVA_RSDF_DIR) / "kinova_default";
  if(fs::exists(defaultDir))
  {
    mc_rtc::log::warning("No RSDF directory exists for variant '{}', falling back to '{}'", variant,
                         defaultDir.string());
    return defaultDir;
  }

  return variantDir;
}

KinovaRobotModule::KinovaRobotModule(bool callib,
                                     ForceSensor force_sensor,
                                     EndEffector end_effector,
                                     bool camera,
                                     Gripper gripper,
                                     bool mujoco)
: mc_rbdyn::RobotModule(KINOVA_DESCRIPTION_PATH, kinovaVariant(force_sensor, end_effector, camera, gripper))
{
  const auto variant = kinovaVariant(force_sensor, end_effector, camera, gripper);

  if(callib && !supportsCallib(force_sensor, end_effector))
  {
    throw std::invalid_argument("KinovaRobotModule callib mode requires a Bota variant with a mounted end effector");
  }

  mc_rtc::log::success("KinovaRobotModule loaded with name: {}", name);
  if(callib)
  {
    mc_rtc::log::info("KinovaRobotModule runs in callib mode for variant: '{}'", name);
  }

  urdf_path = fs::path(KINOVA_URDF_DIR) / (name + ".urdf");

  _real_urdf = urdf_path;

  // Makes all the basic initialization that can be done from an URDF file
  init(rbd::parsers::from_urdf_file(urdf_path, true));

  rsdf_dir = kinovaRsdfDir(variant);
  mc_rtc::log::success("KinovaRobotModule using path \"{}\" for rsdf", rsdf_dir);

  _ref_joint_order = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"};

  if(hasGripper(gripper))
  {
    const auto & spec = gripperSpec(gripper);
    if(mujoco)
    {
      _ref_joint_order.insert(_ref_joint_order.end(), spec.refJoints.begin(), spec.refJoints.end());
    }
    else
    {
      _ref_joint_order.push_back(spec.actuatedJoint);
    }
    auto gripperSafety = mc_rbdyn::RobotModule::Gripper::Safety{mujoco ? 0.5 : 0.99, 0.05, 0.05, 1};
    _grippers = {{"gripper", {spec.actuatedJoint}, true, gripperSafety}};
  }

  // Override position, velocity and effort bounds
  auto update_joint_limit = [this](const std::string & name, double limit_low, double limit_up)
  {
    assert(limit_up > 0);
    assert(limit_low < 0);
    assert(_bounds[0].at(name).size() == 1);
    _bounds[0].at(name)[0] = limit_low;
    _bounds[1].at(name)[0] = limit_up;
  };

  if(callib)
  {
    update_joint_limit("joint_2", -2.15, 2.15);
    update_joint_limit("joint_4", -2.45, 0.45);
    update_joint_limit("joint_5", -3.14, 3.14);
    update_joint_limit("joint_6", -2.0, 2.0);
    update_joint_limit("joint_7", -3.14, 3.14);
  }
  else
  {
    update_joint_limit("joint_2", -2.15, 2.15);
    update_joint_limit("joint_4", -2.45, 2.45);
    update_joint_limit("joint_6", -2.0, 2.0);
  }

  auto update_velocity_limit = [this](const std::string & name, double limit)
  {
    assert(limit > 0);
    assert(_bounds[2].at(name).size() == 1);
    _bounds[2].at(name)[0] = -limit;
    _bounds[3].at(name)[0] = limit;
  };
  update_velocity_limit("joint_1", 2.0944);
  update_velocity_limit("joint_2", 2.0944);
  update_velocity_limit("joint_3", 2.0944);
  update_velocity_limit("joint_3", 2.0944);
  update_velocity_limit("joint_4", 2.0944);
  update_velocity_limit("joint_5", 3.049);
  update_velocity_limit("joint_6", 3.049);
  update_velocity_limit("joint_7", 3.049);
  auto update_torque_limit = [this](const std::string & name, double limit)
  {
    assert(limit > 0);
    assert(_bounds[4].at(name).size() == 1);
    _bounds[4].at(name)[0] = -limit;
    _bounds[5].at(name)[0] = limit;
  };
  update_torque_limit("joint_1", 95);
  update_torque_limit("joint_2", 95);
  update_torque_limit("joint_3", 95);
  update_torque_limit("joint_4", 95);
  update_torque_limit("joint_5", 45);
  update_torque_limit("joint_6", 45);
  update_torque_limit("joint_7", 45);

  auto set_gear_ratio = [this](const std::string & name, double gr)
  {
    assert(gr > 0);
    mb.setJointGearRatio(mb.jointIndexByName(name), gr);
  };

  set_gear_ratio("joint_1", 100);
  set_gear_ratio("joint_2", 100);
  set_gear_ratio("joint_3", 100);
  set_gear_ratio("joint_4", 100);
  set_gear_ratio("joint_5", 100);
  set_gear_ratio("joint_6", 100);
  set_gear_ratio("joint_7", 100);

  auto set_rotor_inertia = [this](const std::string & name, double ir)
  {
    assert(ir > 0);
    mb.setJointRotorInertia(mb.jointIndexByName(name), ir);
  };

  const double power = pow(10, -4);
  set_rotor_inertia("joint_1", (double)0.40 * power);
  set_rotor_inertia("joint_2", (double)0.40 * power);
  set_rotor_inertia("joint_3", (double)0.40 * power);
  set_rotor_inertia("joint_4", (double)0.40 * power);
  set_rotor_inertia("joint_5", (double)0.22 * power);
  set_rotor_inertia("joint_6", (double)0.22 * power);
  set_rotor_inertia("joint_7", (double)0.22 * power);

  // Automatically load the convex hulls associated to each body
  fs::path convexPath = fs::path(KINOVA_CONVEX_DIR) / "kinova";
  mc_rtc::log::success("KinovaRobotModule using path \"{}\" for convex", convexPath.string());

  for(const auto & b : mb.bodies())
  {
    auto ch = convexPath / (b.name() + "-ch.txt");
    if(fs::exists(ch))
    {
      _convexHull[b.name()] = {b.name(), ch.string()};
    }
  }

  // Add JointSensors for temperature/current logging
  for(size_t i = 0; i < _ref_joint_order.size(); ++i)
  {
    if(mb.jointIndexByName().count(_ref_joint_order[i]) != 0)
    {
      _jointSensors.push_back(mc_rbdyn::JointSensor(_ref_joint_order[i]));
    }
  }

  // Define a force sensor
  if(hasBota(force_sensor))
  {
    _forceSensors.push_back(mc_rbdyn::ForceSensor("EEForceSensor", "FT_sensor_wrench", sva::PTransformd::Identity()));
    _bodySensors.push_back(mc_rbdyn::BodySensor("Accelerometer", "FT_sensor_imu", sva::PTransformd::Identity()));
  }
  else
  {
    _forceSensors.push_back(mc_rbdyn::ForceSensor("EEForceSensor", "tool_frame", sva::PTransformd::Identity()));
    _bodySensors.push_back(mc_rbdyn::BodySensor("Accelerometer", "tool_frame", sva::PTransformd::Identity()));
  }

  // Clear body sensors
  _bodySensors.clear();

  const double i = 0.03;
  const double s = 0.015;
  const double d = 0.;
  // Define a minimal set of self-collisions
  _minimalSelfCollisions = {{"base_link", "spherical_wrist_1_link", i, s, d},
                            {"shoulder_link", "spherical_wrist_1_link", i, s, d},
                            {"half_arm_1_link", "spherical_wrist_1_link", i, s, d},
                            {"half_arm_2_link", "spherical_wrist_1_link", i, s, d},
                            {"base_link", "spherical_wrist_2_link", i, s, d},
                            {"shoulder_link", "spherical_wrist_2_link", i, s, d},
                            {"half_arm_1_link", "spherical_wrist_2_link", i, s, d},
                            {"half_arm_2_link", "spherical_wrist_2_link", i, s, d},
                            {"base_link", "bracelet_link", i, s, d},
                            {"shoulder_link", "bracelet_link", i, s, d},
                            {"half_arm_1_link", "bracelet_link", i, s, d},
                            {"half_arm_2_link", "bracelet_link", i, s, d}};

  if(force_sensor == ForceSensor::BotaGen0)
  {
    _minimalSelfCollisions.insert(_minimalSelfCollisions.end(), {{"base_link", "FT_adapter", i, s, d},
                                                                 {"shoulder_link", "FT_adapter", i, s, d},
                                                                 {"half_arm_1_link", "FT_adapter", i, s, d},
                                                                 {"half_arm_2_link", "FT_adapter", i, s, d}});
  }

  if(hasBota(force_sensor))
  {
    _minimalSelfCollisions.insert(_minimalSelfCollisions.end(), {{"base_link", "FT_sensor_mounting", i, s, d},
                                                                 {"shoulder_link", "FT_sensor_mounting", i, s, d},
                                                                 {"half_arm_1_link", "FT_sensor_mounting", i, s, d},
                                                                 {"half_arm_2_link", "FT_sensor_mounting", i, s, d}});
  }

  if(end_effector == EndEffector::DS4)
  {
    _minimalSelfCollisions.insert(_minimalSelfCollisions.end(), {{"base_link", "DS4_adapter", i, s, d},
                                                                 {"shoulder_link", "DS4_adapter", i, s, d},
                                                                 {"half_arm_1_link", "DS4_adapter", i, s, d},
                                                                 {"half_arm_2_link", "DS4_adapter", i, s, d}});
  }

  if(end_effector == EndEffector::Plate)
  {
    _minimalSelfCollisions.insert(_minimalSelfCollisions.end(), {{"base_link", "plate", i, s, d},
                                                                 {"shoulder_link", "plate", i, s, d},
                                                                 {"half_arm_1_link", "plate", i, s, d},
                                                                 {"half_arm_2_link", "plate", i, s, d}});
  }

  if(end_effector == EndEffector::Screw)
  {
    _minimalSelfCollisions.insert(_minimalSelfCollisions.end(), {{"base_link", "screw", i, s, d},
                                                                 {"shoulder_link", "screw", i, s, d},
                                                                 {"half_arm_1_link", "screw", i, s, d},
                                                                 {"half_arm_2_link", "screw", i, s, d}});
  }

  if(hasGripper(gripper))
  {
    for(const auto & link : gripperSpec(gripper).collisionLinks)
    {
      _minimalSelfCollisions.insert(_minimalSelfCollisions.end(), {{"base_link", link, i, s, d},
                                                                   {"shoulder_link", link, i, s, d},
                                                                   {"half_arm_1_link", link, i, s, d},
                                                                   {"half_arm_2_link", link, i, s, d}});
    }
  }

  /* Additional self collisions */

  _commonSelfCollisions = _minimalSelfCollisions;

  // Default configuration of the floating base
  _default_attitude = {{1., 0., 0., 0., 0., 0., 0.0}};

  // Default joint configuration, if a joint is omitted the configuration is 0 or the middle point of the limit range if
  // 0 is not a valid configuration
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
