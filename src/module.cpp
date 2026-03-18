#include "kinova.h"

#include <mc_rbdyn/RobotModuleMacros.h>
#include <mc_rtc/logging.h>

extern "C"
{
  ROBOT_MODULE_API void MC_RTC_ROBOT_MODULE(std::vector<std::string> & names)
  {
    names = {"Kinova",          "KinovaBota",           "KinovaBotaDS4",       "KinovaBotaDS4Callib",
             "KinovaBotaPlate", "KinovaBotaPlateCallib", "KinovaBotaScrew",   "KinovaBotaScrewCallib",
             "KinovaBotaGenA",  "KinovaBotaGenADS4",    "KinovaBotaGenADS4Callib",
             "KinovaBotaGenAPlate", "KinovaBotaGenAPlateCallib", "KinovaBotaGenAScrew",
             "KinovaBotaGenAScrewCallib",
             "KinovaBotaGenAGripper", "KinovaBotaGenARobotiq2F85", "KinovaBotaGenARobotiq2F140",
             "KinovaCamera",    "KinovaCameraGripper",  "KinovaGripper",
             "KinovaCameraRobotiq2F85", "KinovaRobotiq2F85",
             "KinovaCameraRobotiq2F140", "KinovaRobotiq2F140"};
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
      return new mc_robots::KinovaRobotModule(false, mc_robots::KinovaRobotModule::ForceSensor::None);
    }
    else if(n == "KinovaBota")
    {
      return new mc_robots::KinovaRobotModule(false, mc_robots::KinovaRobotModule::ForceSensor::BotaGen0);
    }
    else if(n == "KinovaBotaDS4")
    {
      return new mc_robots::KinovaRobotModule(false, mc_robots::KinovaRobotModule::ForceSensor::BotaGen0,
                                              mc_robots::KinovaRobotModule::EndEffector::DS4);
    }
    else if(n == "KinovaBotaPlate")
    {
      return new mc_robots::KinovaRobotModule(false, mc_robots::KinovaRobotModule::ForceSensor::BotaGen0,
                                              mc_robots::KinovaRobotModule::EndEffector::Plate);
    }
    else if(n == "KinovaBotaScrew")
    {
      return new mc_robots::KinovaRobotModule(false, mc_robots::KinovaRobotModule::ForceSensor::BotaGen0,
                                              mc_robots::KinovaRobotModule::EndEffector::Screw);
    }
    else if(n == "KinovaBotaDS4Callib")
    {
      return new mc_robots::KinovaRobotModule(true, mc_robots::KinovaRobotModule::ForceSensor::BotaGen0,
                                              mc_robots::KinovaRobotModule::EndEffector::DS4);
    }
    else if(n == "KinovaBotaPlateCallib")
    {
      return new mc_robots::KinovaRobotModule(true, mc_robots::KinovaRobotModule::ForceSensor::BotaGen0,
                                              mc_robots::KinovaRobotModule::EndEffector::Plate);
    }
    else if(n == "KinovaBotaScrewCallib")
    {
      return new mc_robots::KinovaRobotModule(true, mc_robots::KinovaRobotModule::ForceSensor::BotaGen0,
                                              mc_robots::KinovaRobotModule::EndEffector::Screw);
    }
    else if(n == "KinovaBotaGenA")
    {
      return new mc_robots::KinovaRobotModule(false, mc_robots::KinovaRobotModule::ForceSensor::BotaGenA);
    }
    else if(n == "KinovaBotaGenAGripper" || n == "KinovaBotaGenARobotiq2F85")
    {
      return new mc_robots::KinovaRobotModule(false, mc_robots::KinovaRobotModule::ForceSensor::BotaGenA,
                                              mc_robots::KinovaRobotModule::EndEffector::None, false,
                                              mc_robots::KinovaRobotModule::Gripper::Robotiq2F85);
    }
    else if(n == "KinovaBotaGenARobotiq2F140")
    {
      return new mc_robots::KinovaRobotModule(false, mc_robots::KinovaRobotModule::ForceSensor::BotaGenA,
                                              mc_robots::KinovaRobotModule::EndEffector::None, false,
                                              mc_robots::KinovaRobotModule::Gripper::Robotiq2F140);
    }
    else if(n == "KinovaBotaGenADS4")
    {
      return new mc_robots::KinovaRobotModule(false, mc_robots::KinovaRobotModule::ForceSensor::BotaGenA,
                                              mc_robots::KinovaRobotModule::EndEffector::DS4);
    }
    else if(n == "KinovaBotaGenAPlate")
    {
      return new mc_robots::KinovaRobotModule(false, mc_robots::KinovaRobotModule::ForceSensor::BotaGenA,
                                              mc_robots::KinovaRobotModule::EndEffector::Plate);
    }
    else if(n == "KinovaBotaGenAScrew")
    {
      return new mc_robots::KinovaRobotModule(false, mc_robots::KinovaRobotModule::ForceSensor::BotaGenA,
                                              mc_robots::KinovaRobotModule::EndEffector::Screw);
    }
    else if(n == "KinovaBotaGenADS4Callib")
    {
      return new mc_robots::KinovaRobotModule(true, mc_robots::KinovaRobotModule::ForceSensor::BotaGenA,
                                              mc_robots::KinovaRobotModule::EndEffector::DS4);
    }
    else if(n == "KinovaBotaGenAPlateCallib")
    {
      return new mc_robots::KinovaRobotModule(true, mc_robots::KinovaRobotModule::ForceSensor::BotaGenA,
                                              mc_robots::KinovaRobotModule::EndEffector::Plate);
    }
    else if(n == "KinovaBotaGenAScrewCallib")
    {
      return new mc_robots::KinovaRobotModule(true, mc_robots::KinovaRobotModule::ForceSensor::BotaGenA,
                                              mc_robots::KinovaRobotModule::EndEffector::Screw);
    }
    else if(n == "KinovaCamera")
    {
      return new mc_robots::KinovaRobotModule(false, mc_robots::KinovaRobotModule::ForceSensor::None,
                                              mc_robots::KinovaRobotModule::EndEffector::None, true);
    }
    else if(n == "KinovaCameraGripper" || n == "KinovaCameraRobotiq2F85")
    {
      return new mc_robots::KinovaRobotModule(false, mc_robots::KinovaRobotModule::ForceSensor::None,
                                              mc_robots::KinovaRobotModule::EndEffector::None, true,
                                              mc_robots::KinovaRobotModule::Gripper::Robotiq2F85);
    }
    else if(n == "KinovaGripper" || n == "KinovaRobotiq2F85")
    {
      return new mc_robots::KinovaRobotModule(false, mc_robots::KinovaRobotModule::ForceSensor::None,
                                              mc_robots::KinovaRobotModule::EndEffector::None, false,
                                              mc_robots::KinovaRobotModule::Gripper::Robotiq2F85);
    }
    else if(n == "KinovaCameraRobotiq2F140")
    {
      return new mc_robots::KinovaRobotModule(false, mc_robots::KinovaRobotModule::ForceSensor::None,
                                              mc_robots::KinovaRobotModule::EndEffector::None, true,
                                              mc_robots::KinovaRobotModule::Gripper::Robotiq2F140);
    }
    else if(n == "KinovaRobotiq2F140")
    {
      return new mc_robots::KinovaRobotModule(false, mc_robots::KinovaRobotModule::ForceSensor::None,
                                              mc_robots::KinovaRobotModule::EndEffector::None, false,
                                              mc_robots::KinovaRobotModule::Gripper::Robotiq2F140);
    }
    else
    {
      mc_rtc::log::error("Kinova module cannot create an object of type {}", n);
      return nullptr;
    }
  }
}
