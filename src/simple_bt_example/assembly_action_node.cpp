
#include <memory>
#include <algorithm>

#include <plansys2_executor/ActionExecutorClient.hpp>

#include <ros/ros.h>
//#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class AssembleAction : public plansys2::ActionExecutorClient
{
public:
  AssembleAction(ros::NodeHandle nh, std::string _name)
  : plansys2::ActionExecutorClient(nh, _name, 500ms)
  {
    progress_ = 0.0;
  }

private:
  void do_work()
  {
    if (progress_ < 1.0) {
      progress_ += 0.05;
      send_feedback(progress_, "Assemble running");
    } else {
      finish(true, 1.0, "Assemble completed");

      progress_ = 0.0;
      std::cout << std::endl;
    }

    std::cout << "\r\e[K" << std::flush;
    std::cout << "Assemble ... [" << std::min(100.0, progress_ * 100.0) << "%]  " <<
      std::flush;
  }

  float progress_;
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "assembly_action_node");
  std::string name = ros::this_node::getName();
  if(name.empty() || (name.size() == 1 && name[0] == '/') )
    name = "default";
  else if(name[0] == '/')
    name = name.substr(1);

  ros::NodeHandle nh(name);

  
  std::shared_ptr<AssembleAction> node;
  node.reset( new AssembleAction(nh, name) );

  // Getting name
  std::string node_name = std::string(node->get_name());
  printf("\t\t **** name of AssembleAction: %s Node name: %s \n", name.c_str(), node_name.c_str());

  //node->set_parameter(rclcpp::Parameter("action_name", "assemble"));
  node->trigger_transition(ros::lifecycle::CONFIGURE);

  ros::spin();

  return 0;
}
