#pragma once

#include <string>

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

namespace ros2_planning_system_examples
{

class Recharge : public BT::ActionNodeBase
{
public:
  explicit Recharge(
    const std::string & xml_tag_name,
    const BT::NodeConfig & conf);

  void halt();
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList({});
  }

private:
  int counter_;
};

} 
