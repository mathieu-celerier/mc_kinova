#include "KinovaRobotModule.h"

#include <RBDyn/parsers/urdf.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

namespace
{

// This is set by CMake, see CMakeLists.txt
static const std::string KINOVA_DESCRIPTION_PATH = "@KORTEX_DESCRIPTION_PATH@";

} // namespace

namespace mc_robots
{

KinovaRobotModule::KinovaRobotModule() : mc_rbdyn::RobotModule(KINOVA_DESCRIPTION_PATH, "kinova")
{
  mc_rtc::log::success("KinovaRobotModule loaded with name: {}", name);
  urdf_path = path + "/arms/gen3/7dof/urdf/GEN3_URDF_V12.urdf";
  _real_urdf = urdf_path;
  // Makes all the basic initialization that can be done from an URDF file
  init(rbd::parsers::from_urdf_file(urdf_path, true));

  // Automatically load the convex hulls associated to each body
  std::string convexPath = "@PROJECT_SOURCE_DIR@/convex/" + name + "/";
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
  _forceSensors.push_back(mc_rbdyn::ForceSensor("EEForceSensor", "EndEffector_Link", sva::PTransformd::Identity()));

  // Clear body sensors
  _bodySensors.clear();

  const double i = 0.03;
  const double s = 0.015;
  const double d = 0.;
  // Define a minimal set of self-collisions
  _minimalSelfCollisions = {
      {"base_link", "spherical_wrist_1_link", i, s, d},
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
      {"half_arm_2_link", "bracelet_link", i, s, d}
  };

  /* Additional self collisions */

  _commonSelfCollisions = _minimalSelfCollisions;

  // Define simple grippers
  // _grippers = {{"l_gripper", {"L_UTHUMB"}, true}, {"r_gripper", {"R_UTHUMB"}, false}};

  // Default configuration of the floating base
  _default_attitude = {{1., 0., 0., 0., 0., 0., 0.0}};

  // Default joint configuration, if a joint is omitted the configuration is 0 or the middle point of the limit range if
  // 0 is not a valid configuration
  _stance["Actuator1"] = {-1.57};
  _stance["Actuator2"] = {-0.35};
  _stance["Actuator3"] = {3.14};
  _stance["Actuator4"] = {-2.0};
  _stance["Actuator5"] = {0.0};
  _stance["Actuator6"] = {-1.0};
  _stance["Actuator7"] = {1.57};

  mc_rtc::log::success("PandaRobotModule uses urdf_path {}", urdf_path);
}

} // namespace mc_robots

#include <mc_rbdyn/RobotModuleMacros.h>

ROBOT_MODULE_DEFAULT_CONSTRUCTOR("kinova", mc_robots::KinovaRobotModule)
