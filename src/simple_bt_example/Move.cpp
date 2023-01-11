#include <string>
#include <iostream>

#include <ros2_planning_system_examples/simple_bt_example/Move.h>

#include <behaviortree_cpp/behavior_tree.h>

namespace ros2_planning_system_examples
{

Move::Move(
  const std::string & xml_tag_name,
  const BT::NodeConfig & conf)
: BT::ActionNodeBase(xml_tag_name, conf), counter_(0)
{
}

void Move::halt()
{
  std::cout << "Move halt" << std::endl;
}

BT::NodeStatus
Move::tick()
{
  std::string goal;
  getInput<std::string>("goal", goal);
  std::cout << "Move to: " << goal << std::endl;
  
  std::cout << "Move tick " << counter_ << std::endl;

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
  factory.registerNodeType<ros2_planning_system_examples::Move>("Move");
}
