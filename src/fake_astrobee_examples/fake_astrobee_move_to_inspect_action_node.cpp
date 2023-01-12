
#include <memory>
#include <algorithm>

#include <plansys2_executor/ActionExecutorClient.hpp>

#include <ros/ros.h>
//#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class FakeAstrobeeMoveToInspectAction : public plansys2::ActionExecutorClient
{
public:
  FakeAstrobeeMoveToInspectAction(const ros::NodeHandle &nh, std::string _name)
  : plansys2::ActionExecutorClient(nh, _name, 500ms)
  {
    progress_ = 0.0;
    nh.getParam("robot_name", robot_name_);
  }

private:
  void do_work()
  { 
    if (progress_ < 1.0) {
      progress_ += 0.05;
      send_feedback(progress_, "Move and Inspect running");
    } else {
      finish(true, 1.0, "Move and Inspect completed");

      progress_ = 0.0;
      std::cout << std::endl;
    }

    std::cout << "\r\e[K" << std::flush;
    std::string from, towards;
    from = get_arguments()[1];
    towards = get_arguments()[2]; 
    std::cout << "\t ** [Move and Inspect] Robot " << robot_name_ <<" moving from " << from << " towards " << towards << " ... [" << std::min(100.0, progress_ * 100.0) << "%]  " <<
      std::flush;
  }

  float progress_;
  std::string robot_name_;
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "fake_astrobee_move_to_inspect_action_node");
  std::string name = ros::this_node::getName();
  if(name.empty() || (name.size() == 1 && name[0] == '/') )
    name = "default";
  else if(name[0] == '/')
    name = name.substr(1);

  ros::NodeHandle nh("~");
  std::shared_ptr<FakeAstrobeeMoveToInspectAction> node;
  node.reset( new FakeAstrobeeMoveToInspectAction(nh, name) );

  // Getting name
  std::string node_name = std::string(node->get_name());

  //node->set_parameter(rclcpp::Parameter("action_name", "move_to_inspect"));
  node->trigger_transition(ros::lifecycle::CONFIGURE);

  ros::spin();

  return 0;
}
