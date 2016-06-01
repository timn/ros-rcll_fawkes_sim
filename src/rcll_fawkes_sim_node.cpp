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
#include <ros/callback_queue.h>

#include <blackboard/remote.h>
#include <blackboard/interface_listener.h>

#include <interfaces/ZoneInterface.h>
#include <interfaces/Position3DInterface.h>
#include <interfaces/TagVisionInterface.h>
#include <interfaces/RobotinoLightInterface.h>
#include <interfaces/NavGraphWithMPSGeneratorInterface.h>
// these are Fawkes' utils
#include <utils/time/time.h>

#include <rcll_fawkes_sim_msgs/ExplorationZoneInfo.h>
#include <rcll_fawkes_sim_msgs/MPSMarkerArray.h>
#include <rcll_fawkes_sim_msgs/MPSLightState.h>
#include <rcll_fawkes_sim_msgs/NavgraphWithMPSGenerate.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <memory>

#define GET_PRIV_PARAM(P)	  \
	{ \
		if (! ros::param::get("~" #P, cfg_ ## P ## _)) { \
			ROS_ERROR("%s: Failed to retrieve parameter " #P ", aborting", ros::this_node::getName().c_str()); \
			exit(-1); \
		} \
	}

std::string  cfg_fawkes_host_;
int          cfg_fawkes_port_;

std::shared_ptr<fawkes::BlackBoard> blackboard_;
class DataPasser;
std::shared_ptr<DataPasser> dp_;

fawkes::ZoneInterface       *ifc_zone_;
fawkes::TagVisionInterface  *ifc_tag_vision_;
std::array<fawkes::Position3DInterface *, 16> ifcs_tag_pos_;
fawkes::RobotinoLightInterface  *ifc_machine_signal_;
fawkes::NavGraphWithMPSGeneratorInterface *ifc_navgraph_gen_;
fawkes::Position3DInterface *ifc_pose_;

ros::Publisher pub_expl_zone_info_;
ros::Publisher pub_mps_marker_array_;
ros::Publisher pub_mps_light_state_;
ros::Publisher pub_pose_;
ros::ServiceServer srv_navgraph_gen_;

int
fawkes_to_ros_light_state(const fawkes::RobotinoLightInterface::LightState &state)
{
	switch (state) {
	case fawkes::RobotinoLightInterface::ON:
		return rcll_fawkes_sim_msgs::MPSLightState::LIGHT_STATE_ON;
	case fawkes::RobotinoLightInterface::OFF:
		return rcll_fawkes_sim_msgs::MPSLightState::LIGHT_STATE_OFF;
	case fawkes::RobotinoLightInterface::BLINKING:
		return rcll_fawkes_sim_msgs::MPSLightState::LIGHT_STATE_BLINKING;
	}
	return rcll_fawkes_sim_msgs::MPSLightState::LIGHT_STATE_UNKNOWN;
}

fawkes::NavGraphWithMPSGeneratorInterface::Side
ros_to_fawkes_mps_side(int side)
{
	if (side == rcll_fawkes_sim_msgs::NavgraphMPSStation::SIDE_INPUT) {
		return fawkes::NavGraphWithMPSGeneratorInterface::INPUT;
	} else {
		return fawkes::NavGraphWithMPSGeneratorInterface::OUTPUT;
	}
}

bool
srv_cb_navgraph_gen(rcll_fawkes_sim_msgs::NavgraphWithMPSGenerate::Request  &req,
                    rcll_fawkes_sim_msgs::NavgraphWithMPSGenerate::Response &res)
{
	if (! blackboard_->is_alive()) {
		res.ok = false;
		res.error_msg = "Blackboard is not connected";
		return true;
	}
	
	if (! ifc_navgraph_gen_->has_writer()) {
		res.ok = false;
		res.error_msg = "No writer for navgraph generator with MPS interface";
		return true;
	}

	std::queue<fawkes::Message *> msgs;
	
	ifc_navgraph_gen_->msgq_enqueue(new fawkes::NavGraphWithMPSGeneratorInterface::ClearMessage());

	for (size_t i = 0; i < req.mps_stations.size(); ++i) {
		const rcll_fawkes_sim_msgs::NavgraphMPSStation &mps = req.mps_stations[i];
		fawkes::NavGraphWithMPSGeneratorInterface::UpdateStationByTagMessage *upm =
			new fawkes::NavGraphWithMPSGeneratorInterface::UpdateStationByTagMessage();
#ifdef HAVE_OLD_NAVGRAPH_GENMPS_INTERFACE
		upm->set_id(mps.name.c_str());
#else
		upm->set_name(mps.name.c_str());
#endif
		upm->set_side(ros_to_fawkes_mps_side(mps.marker_side));
		upm->set_frame(mps.marker_frame.c_str());
		upm->set_tag_translation(0, mps.marker_pose.position.x);
		upm->set_tag_translation(1, mps.marker_pose.position.y);
		upm->set_tag_translation(2, mps.marker_pose.position.z);
		upm->set_tag_rotation(0, mps.marker_pose.orientation.x);
		upm->set_tag_rotation(1, mps.marker_pose.orientation.y);
		upm->set_tag_rotation(2, mps.marker_pose.orientation.z);
		upm->set_tag_rotation(3, mps.marker_pose.orientation.w);
		msgs.push(upm);
	}

	{
		fawkes::NavGraphWithMPSGeneratorInterface::SetExplorationZonesMessage *sezm =
			new fawkes::NavGraphWithMPSGeneratorInterface::SetExplorationZonesMessage();
		bool zones[sezm->maxlenof_zones()];
		for (int i = 0; i < sezm->maxlenof_zones(); ++i) zones[i] = false;
		for (size_t i = 0; i < req.exploration_zones.size(); ++i) {
			int zone_idx = req.exploration_zones[i] - 1;
			if (zone_idx < 0 || zone_idx > sezm->maxlenof_zones()) {
				delete sezm;
				res.ok = false;
				res.error_msg = "Exploration zone index out of bounds";
				return true;
			}
			zones[zone_idx] = true;
		}
		sezm->set_zones(zones);
		msgs.push(sezm);
	}

	{
		fawkes::NavGraphWithMPSGeneratorInterface::SetWaitZonesMessage *swzm =
			new fawkes::NavGraphWithMPSGeneratorInterface::SetWaitZonesMessage();
		bool zones[swzm->maxlenof_zones()];
		for (int i = 0; i < swzm->maxlenof_zones(); ++i) zones[i] = false;
		for (size_t i = 0; i < req.wait_zones.size(); ++i) {
			int zone_idx = req.wait_zones[i] - 1;
			if (zone_idx < 0 || zone_idx > swzm->maxlenof_zones()) {
				delete swzm;
				res.ok = false;
				res.error_msg = "Wait zone index out of bounds";
				return true;
			}
			zones[zone_idx] = true;
		}
		swzm->set_zones(zones);
		msgs.push(swzm);
	}

	while (! msgs.empty()) {
		fawkes::Message *m = msgs.front();
		ifc_navgraph_gen_->msgq_enqueue(m);
		msgs.pop();
	}

	fawkes::NavGraphWithMPSGeneratorInterface::ComputeMessage *compmsg =
		new fawkes::NavGraphWithMPSGeneratorInterface::ComputeMessage();
	compmsg->ref();
	ifc_navgraph_gen_->msgq_enqueue(compmsg);
	unsigned int msgid = compmsg->id();
	compmsg->unref();
	bool finished = false;
	while (! finished && ifc_navgraph_gen_->has_writer()) {
		ifc_navgraph_gen_->read();
		if (ifc_navgraph_gen_->msgid() == msgid && ifc_navgraph_gen_->is_final()) {
			res.ok = true;
			finished = true;
		}
		ros::Duration(0.1 /* sec */).sleep();
	}

	if (finished) {
		res.ok = true;
	} else {
		res.ok = false;
		res.error_msg = "Navgraph generator with MPS interface writer disappeared while waiting";
	}

	return true;
}

class DataPasser : public fawkes::BlackBoardInterfaceListener
{
 public:
	DataPasser() : BlackBoardInterfaceListener("RCLL_ROS_DataPasser") {}
	
	void add(fawkes::Interface *interface)
	{
		ROS_INFO("%s: Adding data interface %s", ros::this_node::getName().c_str(), interface->uid());
		bbil_add_data_interface(interface);
	}
	
	virtual void bb_interface_data_changed(fawkes::Interface *interface) throw()
	{
		interface->read();
		if (strcmp(interface->uid(), ifc_pose_->uid()) == 0) {
			const fawkes::Time *ts = ifc_pose_->timestamp();
			geometry_msgs::PoseWithCovarianceStamped p;
			p.header.frame_id = ifc_pose_->frame();
			p.header.stamp.sec = ts->get_sec();
			p.header.stamp.nsec = ts->get_nsec();
			p.pose.pose.position.x = ifc_pose_->translation(0);
			p.pose.pose.position.y = ifc_pose_->translation(1);
			p.pose.pose.orientation.x = ifc_pose_->rotation(0);
			p.pose.pose.orientation.y = ifc_pose_->rotation(1);
			p.pose.pose.orientation.z = ifc_pose_->rotation(2);
			p.pose.pose.orientation.w = ifc_pose_->rotation(3);

			for (int i = 0; i < ifc_pose_->maxlenof_covariance(); ++i) {
				p.pose.covariance[i] = ifc_pose_->covariance(i);
			}
			pub_pose_.publish(p);
			
		} else if (strcmp(interface->uid(), ifc_zone_->uid()) == 0) {
			rcll_fawkes_sim_msgs::ExplorationZoneInfo rezi;
			rezi.result = rcll_fawkes_sim_msgs::ExplorationZoneInfo::MPS_IN_ZONE_UNKNOWN;
			switch (ifc_zone_->search_state()) {
			case fawkes::ZoneInterface::NO:
				rezi.result = rcll_fawkes_sim_msgs::ExplorationZoneInfo::MPS_IN_ZONE_NO;
				break;
			case fawkes::ZoneInterface::YES:
				rezi.result = rcll_fawkes_sim_msgs::ExplorationZoneInfo::MPS_IN_ZONE_YES;
				break;
			case fawkes::ZoneInterface::MAYBE:
				rezi.result = rcll_fawkes_sim_msgs::ExplorationZoneInfo::MPS_IN_ZONE_MAYBE;
				break;
			}
			rezi.marker_id = ifc_zone_->tag_id();
			ROS_DEBUG("%s: Publishing zone info, marker ID %i", ros::this_node::getName().c_str(), ifc_zone_->tag_id());
			pub_expl_zone_info_.publish(rezi);
		} else if (strcmp(interface->uid(), ifc_tag_vision_->uid()) == 0) {
			rcll_fawkes_sim_msgs::MPSMarkerArray rma;
			for (size_t i = 0; i < std::min(ifc_tag_vision_->maxlenof_tag_id(), ifcs_tag_pos_.size()); ++i) {
				if (ifc_tag_vision_->tag_id(i) > 0) {
					ifcs_tag_pos_[i]->read();
					const fawkes::Time *ts = ifcs_tag_pos_[i]->timestamp();
					rcll_fawkes_sim_msgs::MPSMarker rm;
					rm.id = ifc_tag_vision_->tag_id(i);
					rm.pose.header.stamp.sec = ts->get_sec();
					rm.pose.header.stamp.nsec = ts->get_nsec();
					rm.pose.header.frame_id = ifcs_tag_pos_[i]->frame();
					rm.pose.name = ifcs_tag_pos_[i]->id();
					rm.pose.visibility_history = ifcs_tag_pos_[i]->visibility_history();
					rm.pose.pose.position.x = ifcs_tag_pos_[i]->translation(0);
					rm.pose.pose.position.y = ifcs_tag_pos_[i]->translation(1);
					rm.pose.pose.position.z = ifcs_tag_pos_[i]->translation(2);
					rm.pose.pose.orientation.x = ifcs_tag_pos_[i]->rotation(0);
					rm.pose.pose.orientation.y = ifcs_tag_pos_[i]->rotation(1);
					rm.pose.pose.orientation.z = ifcs_tag_pos_[i]->rotation(2);
					rm.pose.pose.orientation.w = ifcs_tag_pos_[i]->rotation(3);
					rma.markers.push_back(rm);
				}
			}
			pub_mps_marker_array_.publish(rma);
		} else if (strcmp(interface->uid(), ifc_machine_signal_->uid()) == 0) {
			rcll_fawkes_sim_msgs::MPSLightState rls;
			rls.ready = ifc_machine_signal_->is_ready();
			rls.light_state_red = fawkes_to_ros_light_state(ifc_machine_signal_->red());
			rls.light_state_yellow = fawkes_to_ros_light_state(ifc_machine_signal_->yellow());
			rls.light_state_green = fawkes_to_ros_light_state(ifc_machine_signal_->green());
			rls.visibility_history = ifc_machine_signal_->visibility_history();
			pub_mps_light_state_.publish(rls);
		}

	}
};


void
init(ros::NodeHandle &n)
{
	if (! blackboard_)  return;

	dp_ = std::make_shared<DataPasser>();

	ifc_zone_ = blackboard_->open_for_reading<fawkes::ZoneInterface>("/explore-zone/info");
	ifc_tag_vision_ = blackboard_->open_for_reading<fawkes::TagVisionInterface>("/tag-vision/info");
	for (size_t i = 0; i < std::min(ifc_tag_vision_->maxlenof_tag_id(), ifcs_tag_pos_.size()); ++i) {
		std::string ifc_id = "/tag-vision/" + std::to_string(i);
		ifcs_tag_pos_[i] = blackboard_->open_for_reading<fawkes::Position3DInterface>(ifc_id.c_str());
		// do not listen for events on these interfaces, we read them
		// whenever the tag vision info interface changes
	}
	ifc_machine_signal_ = blackboard_->open_for_reading<fawkes::RobotinoLightInterface>("/machine-signal/best");
	ifc_navgraph_gen_ =
		blackboard_->open_for_reading<fawkes::NavGraphWithMPSGeneratorInterface>("/navgraph-generator-mps");
	ifc_pose_ = blackboard_->open_for_reading<fawkes::Position3DInterface>("Pose");
	
	dp_->add(ifc_zone_);
	dp_->add(ifc_tag_vision_);
	dp_->add(ifc_machine_signal_);
	dp_->add(ifc_pose_);

	// Setup ROS topics
	pub_expl_zone_info_ =
		n.advertise<rcll_fawkes_sim_msgs::ExplorationZoneInfo>("rcll_sim/explore_zone_info", 10);
	pub_mps_marker_array_ =
		n.advertise<rcll_fawkes_sim_msgs::MPSMarkerArray>("rcll_sim/mps_marker_array", 10);
	pub_mps_light_state_ =
		n.advertise<rcll_fawkes_sim_msgs::MPSLightState>("rcll_sim/mps_light_state", 10);
	pub_pose_ =
		n.advertise<geometry_msgs::PoseWithCovarianceStamped>("rcll_sim/amcl_pose", 10);

	blackboard_->register_listener(dp_.get(), fawkes::BlackBoard::BBIL_FLAG_DATA);

  // provide services
  srv_navgraph_gen_ = n.advertiseService("rcll_sim/navgraph_generate", srv_cb_navgraph_gen);
}

int
main(int argc, char **argv)
{
	ros::init(argc, argv, "rcll_fawkes_sim");

	ros::NodeHandle n;

	// Parameter parsing	
	GET_PRIV_PARAM(fawkes_host);
	GET_PRIV_PARAM(fawkes_port);

	try {
		blackboard_ =
			std::make_shared<fawkes::RemoteBlackBoard>(cfg_fawkes_host_.c_str(), cfg_fawkes_port_);
		init(n);
	} catch (fawkes::Exception &e) {
		ROS_WARN("%s: Initial connection request failed, will keep trying", ros::this_node::getName().c_str());
	}

	while (ros::ok()) {
		if (!blackboard_) {
			try {
				blackboard_ =
					std::make_shared<fawkes::RemoteBlackBoard>(cfg_fawkes_host_.c_str(), cfg_fawkes_port_);
				ROS_INFO("%s: Blackboard connected", ros::this_node::getName().c_str());
				init(n);
			} catch (fawkes::Exception &e) {}
		} else if (! blackboard_->is_alive()) {
			if (blackboard_->try_aliveness_restore()) {
				ROS_INFO("%s: Blackboard re-connected", ros::this_node::getName().c_str());
			}
		}
		
		ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
	}

  blackboard_->unregister_listener(dp_.get());
  
	return 0;
}
