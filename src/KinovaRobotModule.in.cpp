#include "KinovaRobotModule.h"

#include <RBDyn/parsers/urdf.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

namespace
{

// This is set by CMake, see CMakeLists.txt
static const std::string KINOVA_DESCRIPTION_PATH = "@KORTEX_DESCRIPTION_PATH@";
static const std::string KINOVA_URDF_PATH = "@KINOVA_URDF_PATH@";

} // namespace

namespace mc_robots
{

KinovaRobotModule::KinovaRobotModule() : mc_rbdyn::RobotModule(KINOVA_DESCRIPTION_PATH, "kinova")
{
  mc_rtc::log::success("KinovaRobotModule loaded with name: {}", name);
  urdf_path = KINOVA_URDF_PATH;
  _real_urdf = urdf_path;
  // Makes all the basic initialization that can be done from an URDF file
  init(rbd::parsers::from_urdf_file(urdf_path, true));

  // Automatically load the convex hulls associated to each body
  std::string convexPath = "@CMAKE_INSTALL_FULL_DATADIR@/mc_kinova/convex/" + name + "/";
  bfs::path p(convexPath);
  if(bfs::exists(p) && bfs::is_directory(p))
  {
    std::vector<bfs::path> files;
    std::copy(bfs::directory_iterator(p), bfs::directory_iterator(), std::back_inserter(files));
    for(const bfs::path & file : files)
    {
      size_t off = file.filename().string().rfind("-ch.txt");
      if(off != std::string::npos)
      {
        std::string name = file.filename().string();
        name.replace(off, 7, "");
        _convexHull[name] = std::pair<std::string, std::string>(name, file.string());
      }
    }
  }

  // Define some force sensors
  _forceSensors.push_back(mc_rbdyn::ForceSensor("EEForceSensor", "end_effector_link", sva::PTransformd::Identity()));

  // Define additional joint limits
  // using bound_t = mc_rbdyn::RobotModule::accelerationBounds_t::value_type;
  // bound_t accelerationBoundsUpper;
  // bound_t accelerationBoundsLower;
  // accelerationBoundsLower = {{"joint_1", {-5.2}}, {"joint_2", {-5.2}}, {"joint_3", {-5.2}},
  //                            {"joint_4", {-5.2}}, {"joint_5", {-10}},  {"joint_6", {-10}},
  //                            {"joint_7", {-10}}};
  // accelerationBoundsUpper = {{"joint_1", {5.2}}, {"joint_2", {5.2}}, {"joint_3", {5.2}},
  //                            {"joint_4", {5.2}}, {"joint_5", {10}},  {"joint_6", {10}},
  //                            {"joint_7", {10}}};
  // _accelerationBounds.push_back(accelerationBoundsLower);
  // _accelerationBounds.push_back(accelerationBoundsUpper);
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

  /* Additional self collisions */

  _commonSelfCollisions = _minimalSelfCollisions;

  // Define simple grippers
  // _grippers = {{"l_gripper", {"L_UTHUMB"}, true}, {"r_gripper", {"R_UTHUMB"}, false}};

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

#include <mc_rbdyn/RobotModuleMacros.h>

ROBOT_MODULE_DEFAULT_CONSTRUCTOR("kinova", mc_robots::KinovaRobotModule)
