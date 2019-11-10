// Header file for the class
#include "Homework2.hpp"
#include <tf/tf.h>
#include <stdio.h>
#include <gps_common/GPSFix.h>
#include <gps_common/conversions.h>
#include <dataspeed_ulc_msgs/UlcCmd.h>
#include <cmath>

// Namespace matches ROS package name
namespace homework2 
{  
  // Constructor with global and private node handle arguments
  std_msgs::Float64 Linear_Velocity;
  std_msgs::Float64 calculated_Yaw;
  std_msgs::Float64 Angular_Accel;
  Homework2::Homework2(ros::NodeHandle& n, ros::NodeHandle& pn)
  {
    sub_fix_ = n.subscribe("/vehicle/perfect_gps/enhanced_fix", 1, &Homework2::recvFix, this);
    sub_waypoint_ = n.subscribe("/final_waypoints", 1, &Homework2::recvWaypoints, this);
    srv_.setCallback(boost::bind(&Homework2::reconfig, this, _1, _2));
    
    
    pub_ulc_cmd_ = n.advertise<dataspeed_ulc_msgs::UlcCmd>("/vehicle/ulc_cmd", 1);
    
    
    srv_.setCallback(boost::bind(&Homework2::reconfig, this, _1, _2));
  }
  void Homework2::recvWaypoints(const autoware_msgs::LaneConstPtr& msg)
  {
    //Store UTM coordinates of each waypoint
    utm_waypoints = msg->waypoints;
    
    
  }
  void Homework2::recvFix(const gps_common::GPSFixConstPtr& msg)
  { 
    
    std::vector<autoware_msgs::Waypoint> local_frame_points;
    for (size_t i = 0; i < utm_waypoints.size(); i++) {
      autoware_msgs::Waypoint local_waypoint;
      //do stuff
     
      double current_utm_x ;
      double current_utm_y ;
      std::string current_utm_zone;
      gps_common::LLtoUTM(msg->latitude, msg->longitude, current_utm_y, current_utm_x, current_utm_zone);
      
     
      double heading = (-1)*((msg->track)* M_PI/180) + M_PI/2;
      

      double relative_pos_east = current_utm_x - utm_waypoints[i].pose.pose.position.x;
      double relative_pos_north = current_utm_y - utm_waypoints[i].pose.pose.position.y;

      tf::Transform utm_to_vehicle;
      utm_to_vehicle.setOrigin(tf::Vector3(current_utm_x, current_utm_y, 0.0));
      utm_to_vehicle.setRotation(tf::createQuaternionFromYaw(heading));

      tf::Vector3 ref_utm_vect(utm_waypoints[i].pose.pose.position.x, utm_waypoints[i].pose.pose.position.y, 0.0);
      tf::Vector3 vehicle_frame_vect = utm_to_vehicle.inverse() * ref_utm_vect;

      local_waypoint.pose.pose.position.x = vehicle_frame_vect.x();
      local_waypoint.pose.pose.position.y = vehicle_frame_vect.y();
      local_waypoint.twist.twist.linear.x = utm_waypoints[i].twist.twist.linear.x;
      local_waypoint.twist.twist.angular.z = utm_waypoints[i].twist.twist.angular.z;
      
      if ( vehicle_frame_vect.x() > 10 && vehicle_frame_vect.x() < 15) 
    
        break;
        
        else local_frame_points.push_back(local_waypoint);
        
        calculated_Yaw.data =   (2 * (local_waypoint.pose.pose.position.y)) / ((std::pow(local_waypoint.pose.pose.position.x, 2)) + (std::pow(local_waypoint.pose.pose.position.y, 2)));
        Linear_Velocity.data = local_waypoint.twist.twist.linear.x;
        Angular_Accel.data = local_waypoint.twist.twist.angular.z;
        

    }
      // Declare message structure variable
      dataspeed_ulc_msgs::UlcCmd ulc_cmd_msg;

      // Populate message
      ulc_cmd_msg.enable_pedals = true;
      ulc_cmd_msg.enable_steering = true;
      ulc_cmd_msg.enable_shifting = true;
      ulc_cmd_msg.linear_velocity = Linear_Velocity.data;
      ulc_cmd_msg.yaw_command = calculated_Yaw.data ;
      ulc_cmd_msg.steering_mode = cfg_.steering_mode;
      ulc_cmd_msg.angular_accel = Angular_Accel.data;
      

      pub_ulc_cmd_.publish(ulc_cmd_msg);
  
  }
        

  void Homework2::reconfig(Homework2Config& config, uint32_t level)
  {
    cfg_ = config;
  }

}
