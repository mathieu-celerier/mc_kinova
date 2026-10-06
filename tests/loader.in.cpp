#include <mc_rbdyn/RobotLoader.h>

#include <string>

int main(int argc, char * argv[])
{
  if(argc < 2)
  {
    return 1;
  }
  mc_rbdyn::RobotLoader::clear();
  mc_rbdyn::RobotLoader::update_robot_module_path({std::string(argv[1])});
  if(argc >= 3)
  {
    auto rm = mc_rbdyn::RobotLoader::get_robot_module(std::string(argv[2]));
    return 0;
  }
  auto rm = mc_rbdyn::RobotLoader::get_robot_module(@ROBOT_MODULE_PARAMS@);
  return 0;
}
