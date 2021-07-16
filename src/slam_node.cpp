/**
 * @file slam_node.cpp
 * @brief EKF-SLAM node
 * @author Manas Mehta, mmehta@wpi.edu
 * @date July 2021
 *
 */


#include <ros/ros.h>
#include "ekfSlam.hpp"

int main(int argc, char **argv)
{
	std::string left_cam, right_cam, pose_topic;
	ros::init(argc, argv, "slam_node");  
  
	// Read node parameters
	ros::NodeHandle lnh("~");
	if(!lnh.getParam("left_cam", left_cam))
		left_cam = "/stereo/left";
	if(left_cam[left_cam.length()-1] == '/')
		left_cam.erase(left_cam.length()-1, 1);
	if(!lnh.getParam("right_cam", right_cam))
		right_cam = "/stereo/right";
	if(!lnh.getParam("pose_topic", pose_topic))
        pose_topic = "/pose";
	if(right_cam[right_cam.length()-1] == '/')
		right_cam.erase(right_cam.length()-1, 1);

	// Visual odometry instance
	std::string node_name = "slam_node";
    EkfSlam ekfSlam(node_name, left_cam, right_cam, pose_topic);
	
	// Spin for ever
	ros::spin();
}