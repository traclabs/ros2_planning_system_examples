#include <ros/ros.h>
#include <behaviortree_cpp/action_node.h>

#include <geometry_msgs/PoseStamped.h>
#include <tf/LinearMath/Quaternion.h>

#include <ff_msgs/CommandStamped.h>
#include <ff_msgs/AckStamped.h>
#include <ff_msgs/CommandConstants.h>

namespace ros2_planning_system_examples
{

/**
 * @brief Navigate towards a pose defined in the UR
 */  
class NavigateNode : public BT::ActionNodeBase
{
public:
  NavigateNode(const std::string &_name,
	       const BT::NodeConfig &_conf);
  NavigateNode() = delete;

  BT::NodeStatus tick() override;
  void halt() override;
  
  bool on_tick();

  virtual BT::NodeStatus check_future();
  
  static BT::PortsList providedPorts()
  {
    return {
	    BT::InputPort<std::string>("robot"),
	    BT::InputPort<std::string>("pose_id")
    };
  }

  // Helpers
  bool loadWaypoints( std::string _param);
  void ackCb(ff_msgs::AckStampedConstPtr const &_ack);


protected:
  ros::NodeHandle nh_;
  ros::Subscriber ack_sub_;
  ros::Publisher cmd_pub_;
  ros::Duration action_wait_;

  std::string robot_;
  std::string pose_id_;
  geometry_msgs::PoseStamped pose_;

  // Async stuff
  bool is_navigating_;
  std::future<bool> future_result_;

  // Stuff
  bool motion_cmd_result_;
  bool motion_cmd_return_;
  ros::Time motion_send_time_;
  std::map<std::string, geometry_msgs::PoseStamped> wps_;
  
}; // end of class NavigateNode

  
} // namespace 
