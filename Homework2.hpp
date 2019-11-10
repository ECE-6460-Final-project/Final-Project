// Prevent multiple declarations caused by including
//     this header file from multiple locations
#pragma once

// ROS header
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <homework2/Homework2Config.h>
#include <gps_common/GPSFix.h>
#include <autoware_msgs/Lane.h>
#include <dataspeed_ulc_msgs/UlcCmd.h>
#include <std_msgs/Float64.h>



// Namespace matches ROS package name
namespace homework2 {

  class Homework2 {
    public:
      Homework2(ros::NodeHandle& n, ros::NodeHandle& pn);

    private:
      void reconfig(Homework2Config& config, uint32_t level);
      void recvWaypoints(const autoware_msgs::LaneConstPtr& msg);
      void recvFix(const gps_common ::GPSFixConstPtr& msg);
      //void timerCallback(const ros::TimerEvent& event);

      //ros::Timer timer_;
      ros::Publisher pub_ulc_cmd_;
      
      ros::Subscriber sub_fix_;
      ros::Subscriber sub_waypoint_;
      dynamic_reconfigure::Server<Homework2Config> srv_;
      Homework2Config cfg_;


      std::vector<autoware_msgs::Waypoint> utm_waypoints;
      std::vector<gps_common::GPSFix> current_utm;

  };

}
