/***************************************************************************
 *  rcll_fawkes_sim_node.cpp - RCLL simulation access through Fawkes
 *
 *  Created: Sun May 29 15:36:18 2016
 *  Copyright  2016  Tim Niemueller [www.niemueller.de]
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include <ros/ros.h>

#include <rcll_ros_msgs/SendBeaconSignal.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

geometry_msgs::PoseWithCovarianceStamped last_pose_;
ros::ServiceClient scl_send_beacon_;

unsigned int seq_num_;

void cb_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	last_pose_ = *msg;
}

void cb_timer(const ros::WallTimerEvent& event)
{
	rcll_ros_msgs::SendBeaconSignal sbs;
	sbs.request.header.seq = ++seq_num_;
	ros::WallTime now = ros::WallTime::now();
	sbs.request.header.stamp.sec  = now.sec;
	sbs.request.header.stamp.nsec = now.nsec;
	sbs.request.pose.pose = last_pose_.pose.pose;
	sbs.request.pose.header = last_pose_.header;
	scl_send_beacon_.call(sbs);
	if (! sbs.response.ok) {
		ROS_WARN("Failed to send beacon: %s", sbs.response.error_msg.c_str());
	}
}



int
main(int argc, char **argv)
{
	ros::init(argc, argv, "rcll_beacon_sender");

	ros::NodeHandle n;

	seq_num_ = 0;

	scl_send_beacon_ =
		n.serviceClient<rcll_ros_msgs::SendBeaconSignal>("rcll/send_beacon", /* persistent */ true);

	ros::Subscriber sub_pose =
		n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("rcll_sim/amcl_pose", 10, cb_pose);

	ros::WallTimer timer =
		n.createWallTimer(ros::WallDuration(1.0), cb_timer);

  ros::spin();
  
	return 0;
}