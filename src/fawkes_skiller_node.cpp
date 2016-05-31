/***************************************************************************
 *  fawkes_skiller_node - Skiller interface similar to Fawkes' ros-skiller
 *
 *  Created: Tue May 31 21:24:09 2016
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
#include <interfaces/SkillerInterface.h>
#include <utils/time/time.h>

#include <actionlib/server/simple_action_server.h>
#include <fawkes_msgs/ExecSkillAction.h>
#include <fawkes_msgs/ExecSkillGoal.h>
#include <fawkes_msgs/ExecSkillActionGoal.h>
#include <fawkes_msgs/SkillStatus.h>

#include <memory>

#include <mutex>

#define GET_PRIV_PARAM(P)	  \
	{ \
		if (! ros::param::get("~" #P, cfg_ ## P ## _)) { \
			ROS_ERROR("Failed to retrieve parameter " #P ", aborting"); \
			exit(-1); \
		} \
	}

using namespace fawkes;

typedef actionlib::ActionServer<fawkes_msgs::ExecSkillAction> SkillerServer;


std::string  cfg_fawkes_host_;
int          cfg_fawkes_port_;

std::shared_ptr<fawkes::BlackBoard> blackboard_;

class RosSkillerNode
{
 public:
	RosSkillerNode(ros::NodeHandle *rn, std::shared_ptr<fawkes::BlackBoard> bb)
		: rosnode(rn), blackboard(bb) {}

	fawkes_msgs::ExecSkillResult
	create_result(std::string errmsg)
	{
		fawkes_msgs::ExecSkillResult result;
		result.errmsg = errmsg;
		return result;
	}


	fawkes_msgs::ExecSkillFeedback
	create_feedback()
	{
		return fawkes_msgs::ExecSkillFeedback();
	}

	void
	action_goal_cb(SkillerServer::GoalHandle goal)
	{
		std::lock_guard<std::mutex> lock(loop_mutex);
		if (exec_running_ && exec_as_) {
			std::string error_msg = "Replaced by new goal";
			as_goal_.setAborted(create_result(error_msg), error_msg);
		}
		as_goal_      = goal;
		goal_         = goal.getGoal()->skillstring;
		exec_request_ = true;
		exec_as_      = true;

		goal.setAccepted();
	}

	void
	action_cancel_cb(SkillerServer::GoalHandle goal)
	{
		std::lock_guard<std::mutex> lock(loop_mutex);
		stop();
		std::string error_msg = "Abort on request";
		goal.setCanceled(create_result(error_msg), error_msg);
	}

	virtual void init()
	{
		exec_request_ = false;
		exec_running_ = false;
		exec_as_      = false;

		try {
			skiller_if_ = blackboard->open_for_reading<SkillerInterface>("Skiller");
		} catch (const Exception &e) {
			ROS_ERROR("Initialization failed, could not open Skiller interface");
			throw;
		}

		server_ = new SkillerServer(*rosnode, "skiller",
		                            boost::bind(&RosSkillerNode::action_goal_cb, this, _1),
		                            boost::bind(&RosSkillerNode::action_cancel_cb, this, _1),
		                            /* auto_start */ false);

		pub_status_ = rosnode->advertise<fawkes_msgs::SkillStatus>("skiller_status", true);

		server_->start();
	}

	virtual void finalize()
	{
		try {
			blackboard->close(skiller_if_);
		} catch (Exception& e) {
			ROS_ERROR("Closing interface failed!");
		}
		delete server_;
	}

	void
	stop()
	{
		if (skiller_if_->exclusive_controller() != skiller_if_->serial()) {
			ROS_WARN("Skill abortion requested, but currently not in control, acquiring");
			return;
		}

		if (skiller_if_->has_writer())
			skiller_if_->msgq_enqueue(new SkillerInterface::StopExecMessage());
		if (exec_as_) {
			std::string error_msg = "Abort on request";
			as_goal_.setAborted(create_result(error_msg), error_msg);
		}
		skiller_if_->msgq_enqueue(new SkillerInterface::ReleaseControlMessage());
		exec_running_ = false;
	}

	void
	loop()
	{
		std::lock_guard<std::mutex> lock(loop_mutex);
		skiller_if_->read();

		// currently idle, release skiller control
		if (!exec_running_ && !exec_request_
		    && skiller_if_->exclusive_controller() == skiller_if_->serial()) {
			ROS_INFO("No skill running and no skill requested, releasing control");
			skiller_if_->msgq_enqueue(new SkillerInterface::ReleaseControlMessage());
			return;
		}

		if (exec_request_) {
			if (!skiller_if_->has_writer()) {
				ROS_WARN("no writer for skiller, cannot execute skill");
				stop();
				return;
			}

			if (skiller_if_->exclusive_controller() != skiller_if_->serial()) {
				// we need the skiller control, acquire it first
				ROS_INFO("Skill execution requested, but currently not in control");
				skiller_if_->msgq_enqueue(new SkillerInterface::AcquireControlMessage());
				return;
			}
			exec_request_ = false;

			SkillerInterface::ExecSkillMessage *msg =
				new SkillerInterface::ExecSkillMessage(goal_.c_str());
			msg->ref();

			ROS_INFO("Creating goal '%s'", goal_.c_str());

			try {
				skiller_if_->msgq_enqueue(msg);
				exec_running_ = true;
				exec_msgid_ = msg->id();
				exec_skill_string_ = msg->skill_string();
				loops_waited_ = 0;
			} catch (Exception &e) {
				ROS_WARN("Failed to execute skill, exception %s", e.what_no_backtrace());
			}
			msg->unref();

		} else if (exec_running_) {
			if (exec_as_) as_goal_.publishFeedback(create_feedback());

			if (skiller_if_->status() == SkillerInterface::S_INACTIVE ||
			    skiller_if_->msgid() != exec_msgid_)
			{
				// wait three loops, maybe the skiller will start
				//ROS_INFO("Should be executing skill, but skiller is inactive or processing other message");
				++loops_waited_;
				if (loops_waited_ >= 5) {
					// give up and abort
					ROS_WARN("Skiller doesn't start, aborting");
					std::string error_msg =  "Skiller doesn't start";
					if (exec_as_) as_goal_.setAborted(create_result(error_msg), error_msg);
					exec_running_ = false;
				}
			}
			else if (skiller_if_->status() != SkillerInterface::S_RUNNING) {
				exec_running_ = false;
				if (exec_as_ && exec_skill_string_ == skiller_if_->skill_string()) {
					if (skiller_if_->status() == SkillerInterface::S_FINAL) {
						std::string error_msg = "Skill executed";
						as_goal_.setSucceeded(create_result(error_msg), error_msg);
					}
					else if (skiller_if_->status() == SkillerInterface::S_FAILED){
						std::string error_msg = "Failed to execute skill";
						char * tmp;
						if (asprintf(&tmp, "Failed to execute skill, error: %s", skiller_if_->error()) != -1) {
							error_msg = tmp;
							free(tmp);
						}
						as_goal_.setAborted(create_result(error_msg), error_msg);
					}
					else {
						ROS_WARN("Don't know what happened, but not running");
					}
				}
			}
		}

		if (skiller_if_->changed()) {
			fawkes_msgs::SkillStatus msg;
			const Time *time = skiller_if_->timestamp();
			msg.stamp        = ros::Time(time->get_sec(), time->get_nsec());
			msg.skill_string = skiller_if_->skill_string();
			msg.error        = skiller_if_->error();
			msg.status       = skiller_if_->status();
			pub_status_.publish(msg);
		}
	}

 private:
	ros::NodeHandle *rosnode;
	std::shared_ptr<fawkes::BlackBoard> blackboard;

  std::mutex loop_mutex;

  fawkes::SkillerInterface *skiller_if_;

  SkillerServer   *server_;
  ros::Publisher   pub_status_;

  SkillerServer::GoalHandle as_goal_;
  std::string goal_;

  bool         exec_as_;
  bool         exec_request_;
  bool         exec_running_;
  unsigned int exec_msgid_;
  std::string exec_skill_string_;
  unsigned int loops_waited_;
};


int
main(int argc, char **argv)
{
	ros::init(argc, argv, "fawkes_skiller_forward");

	ros::NodeHandle n;

	// Parameter parsing	
	GET_PRIV_PARAM(fawkes_host);
	GET_PRIV_PARAM(fawkes_port);

	blackboard_ =
		std::make_shared<fawkes::RemoteBlackBoard>(cfg_fawkes_host_.c_str(), cfg_fawkes_port_);


	RosSkillerNode ros_skiller_node(&n, blackboard_);
	ros_skiller_node.init();
	
	while (ros::ok()) {
		ros_skiller_node.loop();
		ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
	}

	ros_skiller_node.finalize();

	return 0;
}
