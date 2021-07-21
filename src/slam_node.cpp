/**
 * @file slam_node.cpp
 * @brief EKF-SLAM node
 * @author Manas Mehta, mmehta@wpi.edu
 * @date July 2021
 *
 */


#include <ros/ros.h>
#include "ekfSlam.hpp"

int main(int argc, char **argv) {

	std::string pose_topic, p2d_topic;
	ros::init(argc, argv, "slam_node");  
  
	// Read node parameters
	ros::NodeHandle lnh("~");
	if(!lnh.getParam("pose_topic", pose_topic))
        pose_topic = "/pose";
	if(!lnh.getParam("p2d_topic", p2d_topic))
        p2d_topic = "/p2d";

	// ekf slam instance
	std::string node_name = "slam_node";
    EkfSlam ekfSlam(node_name, pose_topic, p2d_topic);
	
	// Spin for ever
	ros::spin();
}