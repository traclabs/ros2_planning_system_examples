
#include <ros2_planning_system_examples/astrobee_examples/navigate_node.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace ros2_planning_system_examples
{

NavigateNode::NavigateNode(const std::string &_name,
			   const BT::NodeConfig &_conf)
  : BT::ActionNodeBase(_name, _conf)
{
  // Initialize params
  is_navigating_ = false;
  action_wait_ = ros::Duration(30.0);

}

  
bool NavigateNode::on_tick() {

  if(!getInput("robot", robot_))
    return false;
  if(!getInput("pose_id", pose_id_))
    return false;
  
  ROS_INFO("** Robot: %s -- pose id: %s \n",
	   robot_.c_str(), pose_id_.c_str());
  
  // Read pose from ROS param
  if(!loadWaypoints("/waypoints"))
    return false;

  if(wps_.find(pose_id_) == wps_.end())
  {
    ROS_ERROR("Waypoint %s not found", pose_id_.c_str());
    return false;
  }

  pose_ = wps_[pose_id_];
  
  // Set up publisher and subscriber
  std::string ack_name = "/" + robot_ + "/mgt/ack";
  ack_sub_ = nh_.subscribe(ack_name, 10, &NavigateNode::ackCb, this);
  std::string cmd_name = "/" + robot_ + "/command";
  cmd_pub_ = nh_.advertise<ff_msgs::CommandStamped>(cmd_name, 10, true);
  
  return true;
}

  
  
BT::NodeStatus NavigateNode::tick() {
  
  if(!is_navigating_)
  {
    // Do whatever is needed
    if(!on_tick())
      BT::NodeStatus::FAILURE;
    future_result_ = std::async(std::launch::async, [this]() {

						      ///////////
						      // Send command
						      ff_msgs::CommandStamped cmd;
						      cmd.header.stamp = ros::Time::now();
						      cmd.subsys_name = "Astrobee";
						      
						      cmd.cmd_name = ff_msgs::CommandConstants::CMD_NAME_SIMPLE_MOVE6DOF;
						      cmd.cmd_id = ff_msgs::CommandConstants::CMD_NAME_SIMPLE_MOVE6DOF;
						      
						      // Move command has 4 arguements
						      cmd.args.resize(4);
						      
						      // Set frame
						      cmd.args[0].data_type = ff_msgs::CommandArg::DATA_TYPE_STRING;
						      cmd.args[0].s = pose_.header.frame_id;

						      // Set tolerance. Currently not used but needs to be in the command
						      cmd.args[2].data_type = ff_msgs::CommandArg::DATA_TYPE_VEC3d;
						      cmd.args[2].vec3d[0] = 0;
						      cmd.args[2].vec3d[1] = 0;
						      cmd.args[2].vec3d[2] = 0;
						      
						      cmd.args[1].data_type = ff_msgs::CommandArg::DATA_TYPE_VEC3d;
						      cmd.args[1].vec3d[0] = pose_.pose.position.x;
						      cmd.args[1].vec3d[1] = pose_.pose.position.y;
						      cmd.args[1].vec3d[2] = pose_.pose.position.z;

						      // Parse and set the attitude - roll, pitch then yaw
						      cmd.args[3].data_type = ff_msgs::CommandArg::DATA_TYPE_MAT33f;
						      
						      cmd.args[3].mat33f[0] = pose_.pose.orientation.x;
						      cmd.args[3].mat33f[1] = pose_.pose.orientation.y;
						      cmd.args[3].mat33f[2] = pose_.pose.orientation.z;
						      cmd.args[3].mat33f[3] = pose_.pose.orientation.w;
						      
						      // Publish
						      motion_cmd_result_ = false;
						      motion_cmd_return_ = false;
						      motion_send_time_ = ros::Time::now();
						      
						      cmd_pub_.publish(cmd);

						      ros::Rate r(0.5);
						      while(!motion_cmd_return_)
						      {
							r.sleep();
							ros::spinOnce();
						      }

						      return motion_cmd_result_;
						      ///////////
						    });
    
    is_navigating_ = true;
  }    // if !is_navigating_

  return check_future();      
}

/**
 * @brief Stop execution
 */  
void NavigateNode::halt()
{
  is_navigating_ = false;
  resetStatus();
}
  

/**
 * @brief check if move/plan/execute is over
 */
BT::NodeStatus NavigateNode::check_future() {
  
  auto timeout = std::chrono::milliseconds(10);
  if(future_result_.valid() && future_result_.wait_for(timeout) == std::future_status::ready)
  {
    is_navigating_ = false;
    if(future_result_.get() == true)
      return BT::NodeStatus::SUCCESS;
    else
      return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;

}


void NavigateNode::ackCb(ff_msgs::AckStampedConstPtr const &_ack)
{
  if (_ack->completed_status.status == ff_msgs::AckCompletedStatus::NOT) {
    return;
  } else if (_ack->completed_status.status == ff_msgs::AckCompletedStatus::OK) {
    motion_cmd_result_ = true;
    printf("Motion command returned successfully! \n");
  }
  else if (_ack->completed_status.status == ff_msgs::AckCompletedStatus::CANCELED) {
    motion_cmd_result_ = false;
    printf("Motion command was cancelled! \n");
  }
  else {
    std::cout << "\n" << _ack->cmd_id << " command failed! " << _ack->message;
    std::cout << "\n";
    motion_cmd_result_ = false;
  }
  
  motion_cmd_return_ = true;
}
  
bool NavigateNode::loadWaypoints( std::string _param)
  {
    // Clear
    wps_.clear();
  
    // Read parameter
    XmlRpc::XmlRpcValue wp_list;
    if (!nh_.getParam(_param, wp_list))
      return false;
    
    if (wp_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("loadWaypoints -- comp_list should be of type array (%d)",
		XmlRpc::XmlRpcValue::TypeArray);
      return false;
    }
    
    ROS_WARN("loadWaypoints: wp_list size: %d", wp_list.size());
    
    for (int i = 0; i < wp_list.size(); ++i)
    {
      if (!wp_list[i].hasMember("name") ||
	  !wp_list[i].hasMember("pos") ||
	  !wp_list[i].hasMember("rot") ||
	  !wp_list[i].hasMember("frame") )
      {
	ROS_ERROR("[%d/%d] Waypoint component is lacking either name, init file or param name",
		  i, (int)wp_list.size());
	continue;
      }

      std::string ni;
      geometry_msgs::PoseStamped pi;
      ni = std::string(wp_list[i]["name"]);
      pi.header.frame_id = std::string(wp_list[i]["frame"]);

      if( wp_list[i]["pos"].getType() != XmlRpc::XmlRpcValue::TypeArray ||
	  wp_list[i]["rot"].getType() != XmlRpc::XmlRpcValue::TypeArray )
      {
	ROS_ERROR("pos and/or rot is not array type for %s",
		  ni.c_str());
	return false;    
      }

      if(wp_list[i]["pos"].size() != 3 || wp_list[i]["rot"].size() != 4 )
      {
	ROS_ERROR("Sizes of pos/rot should be 3 and 4. They are %d and %d ",
		  wp_list[i]["pos"].size(), wp_list[i]["rot"].size());
	return false;
      }
      // Fill position
      pi.pose.position.x = (double)(wp_list[i]["pos"][0]);
      pi.pose.position.y = (double)(wp_list[i]["pos"][1]);
      pi.pose.position.z = (double)(wp_list[i]["pos"][2]);

      // Orientation
      double qx, qy, qz, qw;
      qx = (double)(wp_list[i]["rot"][0]);
      qy = (double)(wp_list[i]["rot"][1]);
      qz = (double)(wp_list[i]["rot"][2]);
      qw = (double)(wp_list[i]["rot"][3]);
      tf::Quaternion qr(qx, qy, qz, qw);
      qr.normalize();
      pi.pose.orientation.x = qr.getX();
      pi.pose.orientation.y = qr.getY();
      pi.pose.orientation.z = qr.getZ();
      pi.pose.orientation.w = qr.getW();            
      
      //Add
      wps_[ni] = pi;
      
    } // end for

    // Show
    ROS_INFO("Number of waypoints loaded: %d \n", wps_.size());
    for(auto wi : wps_)
      ROS_INFO("Loaded waypoint %s ", wi.first.c_str());
    
    return true;
  }



} // namespace plum

#include <behaviortree_cpp/bt_factory.h>
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<ros2_planning_system_examples::NavigateNode>("NavigateNode");
}
