#include <string>
#include <iostream>

#include <ros2_planning_system_examples/simple_bt_example/ApproachObject.h>

#include <behaviortree_cpp/behavior_tree.h>

namespace ros2_planning_system_examples
{

ApproachObject::ApproachObject(
  const std::string & xml_tag_name,
  const BT::NodeConfig & conf)
: BT::ActionNodeBase(xml_tag_name, conf), counter_(0)
{
}

void ApproachObject::halt()
{
  std::cout << "ApproachObject halt" << std::endl;
}

BT::NodeStatus
ApproachObject::tick()
{
  std::cout << "ApproachObject tick " << counter_ << std::endl;

  if (counter_++ < 5) {
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
  factory.registerNodeType<ros2_planning_system_examples::ApproachObject>("ApproachObject");
}
