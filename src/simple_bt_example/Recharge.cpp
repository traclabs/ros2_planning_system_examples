#include <string>
#include <iostream>

#include <ros2_planning_system_examples/simple_bt_example/Recharge.h>

#include <behaviortree_cpp/behavior_tree.h>

namespace ros2_planning_system_examples
{

Recharge::Recharge(
  const std::string & xml_tag_name,
  const BT::NodeConfig & conf)
: BT::ActionNodeBase(xml_tag_name, conf), counter_(0)
{
}

void Recharge::halt()
{
  std::cout << "Recharge halt" << std::endl;
}

BT::NodeStatus
Recharge::tick()
{
  std::cout << "Recharge tick " << counter_ << std::endl;

  if (counter_++ < 10) {
    return BT::NodeStatus::RUNNING;
  } else {
    counter_ = 0;
    return BT::NodeStatus::SUCCESS;
  }
}

}  // namespace plansys2_bt_example

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<ros2_planning_system_examples::Recharge>("Recharge");
}
