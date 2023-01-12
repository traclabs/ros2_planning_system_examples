/**
 * @file debug_go_to_waypoint.cpp
 */
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_broadcaster.h>

#include <ff_msgs/CommandStamped.h>
#include <ff_msgs/AckStamped.h>
#include <ff_msgs/CommandConstants.h>

class DebugWps
{
public:

  DebugWps(ros::NodeHandle _nh, std::string _robot) :
    nh_(_nh),
    robot_(_robot)
  {
    std::string ack_name = "/" + robot_ + "/mgt/ack";
    ack_sub_ = nh_.subscribe(ack_name, 10, &DebugWps::ackCb, this);
    std::string cmd_name = "/" + robot_ + "/command";
    cmd_pub_ = nh_.advertise<ff_msgs::CommandStamped>(cmd_name, 10, true);
  }

  void ackCb(ff_msgs::AckStampedConstPtr const &_ack)
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
  
  bool loadWaypoints( std::string _param)
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

  void update()
  {
    publishWps();
    if(motion_cmd_return_)
    {
      printf("Motion cmd return is true, result is: %d \n",
	     motion_cmd_result_);
      motion_cmd_return_ = false;
    }
  }
  
  void publishWps()
  {
    for(auto wi : wps_)
    {
      tf::Transform transform;
      std::string frame, wp_name;
      frame = wi.second.header.frame_id;
      wp_name = wi.first;
      transform.setOrigin(tf::Vector3(wi.second.pose.position.x,
				      wi.second.pose.position.y,
				      wi.second.pose.position.z));
      transform.setRotation(tf::Quaternion(wi.second.pose.orientation.x,
					   wi.second.pose.orientation.y,
					   wi.second.pose.orientation.z,
					   wi.second.pose.orientation.w));
      br_.sendTransform(tf::StampedTransform(transform,
					     ros::Time::now(),
					     frame, wp_name));
    }
  }

  bool goToWaypoint(std::string _wpn)
  {
    if(wps_.find(_wpn) == wps_.end())
    {
      ROS_ERROR("Error. Not finding waypoint %s", _wpn.c_str());
      return false;
    }
    geometry_msgs::PoseStamped wp;
    wp = wps_[_wpn];
    
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
    cmd.args[0].s = wp.header.frame_id;

    // Set tolerance. Currently not used but needs to be in the command
    cmd.args[2].data_type = ff_msgs::CommandArg::DATA_TYPE_VEC3d;
    cmd.args[2].vec3d[0] = 0;
    cmd.args[2].vec3d[1] = 0;
    cmd.args[2].vec3d[2] = 0;

    cmd.args[1].data_type = ff_msgs::CommandArg::DATA_TYPE_VEC3d;
    cmd.args[1].vec3d[0] = wp.pose.position.x;
    cmd.args[1].vec3d[1] = wp.pose.position.y;
    cmd.args[1].vec3d[2] = wp.pose.position.z;

    // Parse and set the attitude - roll, pitch then yaw
    cmd.args[3].data_type = ff_msgs::CommandArg::DATA_TYPE_MAT33f;

    cmd.args[3].mat33f[0] = wp.pose.orientation.x;
    cmd.args[3].mat33f[1] = wp.pose.orientation.y;
    cmd.args[3].mat33f[2] = wp.pose.orientation.z;
    cmd.args[3].mat33f[3] = wp.pose.orientation.w;

    // Publish
    motion_cmd_result_ = false;
    motion_cmd_return_ = false;
    motion_send_time_ = ros::Time::now();

    cmd_pub_.publish(cmd);

    return true;
  }
  
protected:
  ros::NodeHandle nh_;
  std::string robot_;
  tf::TransformBroadcaster br_;
  ros::Subscriber ack_sub_;
  ros::Publisher cmd_pub_;

  std::map<std::string, geometry_msgs::PoseStamped> wps_;
  
  bool motion_cmd_result_;
  bool motion_cmd_return_;
  ros::Time motion_send_time_;
};  // end of class

/**
 * @funtion main
 */
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "debug_go_to_waypoint");
  ros::NodeHandle nh("~");

  std::string wp = "wp0";
  std::string robot = "bumble";
  
  nh.getParam("robot", robot);
  nh.getParam("wp", wp);
  ROS_INFO("Robot %s going to waypoint %s... \n", robot.c_str(), wp.c_str());
  
  DebugWps dw(nh, robot);
  // 1. Load waypoints
  dw.loadWaypoints("/waypoints");

  // 2. Send motion command
  //dw.goToWaypoint(wp);

  ros::Rate r(0.5);
  while(ros::ok())
  {
    dw.update();
    r.sleep();
    ros::spinOnce();
  }


  return 0;
}
