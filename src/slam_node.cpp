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

	std::string kpt_topic;
	ros::init(argc, argv, "slam_node");  
  
	// Read node parameters
	ros::NodeHandle lnh("~");
	if(!lnh.getParam("kpt_topic", kpt_topic))
        kpt_topic = "/kpt";

	// ekf slam instance
	std::string node_name = "slam_node";
    EkfSlam ekfSlam(node_name, kpt_topic);
	
	// Spin for ever
	ros::spin();
}