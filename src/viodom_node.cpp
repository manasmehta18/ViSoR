/**
 * @file viodom_node.cpp
 * @brief Visual odometry node
 * @author Fernando Caballero, fcaballero@us.es
 * @author Francisco J Perez-Grau, fjperez@catec.aero
 * @date October 2016
 *
Copyright (c) 2016, fcaballero, fjperezgrau
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include "stereodom.hpp"

int main(int argc, char **argv)
{
	std::string left_cam, right_cam, imu_topic, slam_pose;
	ros::init(argc, argv, "viodom_node");  
  
	// Read node parameters
	ros::NodeHandle lnh("~");
	if(!lnh.getParam("left_cam", left_cam))
		left_cam = "/stereo/left";
	if(left_cam[left_cam.length()-1] == '/')
		left_cam.erase(left_cam.length()-1, 1);
	if(!lnh.getParam("right_cam", right_cam))
		right_cam = "/stereo/right";
	if(!lnh.getParam("imu_topic", imu_topic))
        imu_topic = "/imu";
	if(right_cam[right_cam.length()-1] == '/')
		right_cam.erase(right_cam.length()-1, 1);
	if(!lnh.getParam("slam_pose", slam_pose))
        imu_topic = "/pose";

	// Visual odometry instance
	std::string node_name = "viodom_node";
	Stereodom odom(node_name, left_cam, right_cam, imu_topic, slam_pose);
	
	// Spin for ever
	ros::spin();
}
