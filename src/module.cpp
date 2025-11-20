#include "kinova.h"

#include <mc_rbdyn/RobotModuleMacros.h>
#include <mc_rtc/logging.h>

extern "C"
{
  ROBOT_MODULE_API void MC_RTC_ROBOT_MODULE(std::vector<std::string> & names)
  {
    names = {"Kinova", "KinovaCallib", "KinovaBota", "KinovaBotaDS4", "KinovaCamera", "KinovaCameraGripper"};
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
      return new mc_robots::KinovaRobotModule(false, false);
    }
    else if(n == "KinovaBota")
    {
      return new mc_robots::KinovaRobotModule(false, true);
    }
    else if(n == "KinovaCallib")
    {
      return new mc_robots::KinovaRobotModule(true, false);
    }
    else if(n == "KinovaBotaDS4")
    {
      return new mc_robots::KinovaRobotModule(false, true, true);
    }
    else if(n == "KinovaCamera")
    {
      return new mc_robots::KinovaRobotModule(false, false, false, true, false);
    }
    else if(n == "KinovaCameraGripper")
    {
      return new mc_robots::KinovaRobotModule(false, false, false, true, true);
    }
    else
    {
      mc_rtc::log::error("Kinova module cannot create an object of type {}", n);
      return nullptr;
    }
  }
}
