#include "kinova.h"

#include <mc_rbdyn/RobotModuleMacros.h>
#include <mc_rtc/logging.h>

extern "C"
{
  ROBOT_MODULE_API void MC_RTC_ROBOT_MODULE(std::vector<std::string> & names)
  {
    names = {"Kinova",
             "KinovaFloatingBase",

             "KinovaCamera",
             "KinovaCameraFloatingBase",
             "KinovaGripper",
             "KinovaGripperFloatingBase",
             "KinovaCameraGripper",
             "KinovaCameraGripperFloatingBase",

             "KinovaBota",
             "KinovaBotaFloatingBase",
             "KinovaBotaDS4",
             "KinovaBotaDS4FloatingBase",
             "KinovaBotaDS4Callib",
             "KinovaBotaDS4CallibFloatingBase",
             "KinovaBotaPlate",
             "KinovaBotaPlateFloatingBase",
             "KinovaBotaPlateCallib",
             "KinovaBotaPlateCallibFloatingBase",
             "KinovaBotaScrew",
             "KinovaBotaScrewFloatingBase",
             "KinovaBotaScrewCallib",
             "KinovaBotaScrewCallibFloatingBase"};
  }
  ROBOT_MODULE_API void destroy(mc_rbdyn::RobotModule * ptr)
  {
    delete ptr;
  }
  ROBOT_MODULE_API mc_rbdyn::RobotModule * create(const std::string & n)
  {
    ROBOT_MODULE_CHECK_VERSION("Kinova")
    if(n == "Kinova")
    {
      return new mc_robots::KinovaRobotModule("kinova", false, true);
    }

    if(n == "KinovaFloatingBase")
    {
      return new mc_robots::KinovaRobotModule("kinova", false, false);
    }

    if(n == "KinovaCamera")
    {
      return new mc_robots::KinovaRobotModule("kinova_camera", false, true);
    }
    if(n == "KinovaCameraFloatingBase")
    {
      return new mc_robots::KinovaRobotModule("kinova_camera", false, false);
    }
    if(n == "KinovaGripper")
    {
      return new mc_robots::KinovaRobotModule("kinova_gripper", false, true);
    }
    if(n == "KinovaGripperFloatingBase")
    {
      return new mc_robots::KinovaRobotModule("kinova_gripper", false, false);
    }
    if(n == "KinovaCameraGripper")
    {
      return new mc_robots::KinovaRobotModule("kinova_camera_gripper", false, true);
    }
    if(n == "KinovaCameraGripperFloatingBase")
    {
      return new mc_robots::KinovaRobotModule("kinova_camera_gripper", false, false);
    }

    if(n == "KinovaBota")
    {
      return new mc_robots::KinovaRobotModule("kinova_bota", false, true);
    }
    if(n == "KinovaBotaFloatingBase")
    {
      return new mc_robots::KinovaRobotModule("kinova_bota", false, false);
    }
    if(n == "KinovaBotaDS4")
    {
      return new mc_robots::KinovaRobotModule("kinova_bota_ds4", false, true);
    }
    if(n == "KinovaBotaDS4FloatingBase")
    {
      return new mc_robots::KinovaRobotModule("kinova_bota_ds4", false, false);
    }
    if(n == "KinovaBotaDS4Callib")
    {
      return new mc_robots::KinovaRobotModule("kinova_bota_ds4", true, true);
    }
    if(n == "KinovaBotaDS4CallibFloatingBase")
    {
      return new mc_robots::KinovaRobotModule("kinova_bota_ds4", true, false);
    }
    if(n == "KinovaBotaPlate")
    {
      return new mc_robots::KinovaRobotModule("kinova_bota_plate", false, true);
    }
    if(n == "KinovaBotaPlateFloatingBase")
    {
      return new mc_robots::KinovaRobotModule("kinova_bota_plate", false, false);
    }
    if(n == "KinovaBotaPlateCallib")
    {
      return new mc_robots::KinovaRobotModule("kinova_bota_plate", true, true);
    }
    if(n == "KinovaBotaPlateCallibFloatingBase")
    {
      return new mc_robots::KinovaRobotModule("kinova_bota_plate", true, false);
    }
    if(n == "KinovaBotaScrew")
    {
      return new mc_robots::KinovaRobotModule("kinova_bota_screw", false, true);
    }
    if(n == "KinovaBotaScrewFloatingBase")
    {
      return new mc_robots::KinovaRobotModule("kinova_bota_screw", false, false);
    }
    if(n == "KinovaBotaScrewCallib")
    {
      return new mc_robots::KinovaRobotModule("kinova_bota_screw", true, true);
    }
    if(n == "KinovaBotaScrewCallibFloatingBase")
    {
      return new mc_robots::KinovaRobotModule("kinova_bota_screw", true, false);
    }

    mc_rtc::log::error("Kinova module cannot create an object of type {}", n);
    return nullptr;
  }
}
