/***************************************************************************
 *  navgraph_node.cpp - navgraph access from ROS
 *
 *  Created: Thu Mar 09 20:28:10 2017
 *  Copyright  2017  Tim Niemueller [www.niemueller.de]
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

// from Fawkes
#include <core/exception.h>
#include <core/utils/lockptr.h>
#include <core/threading/mutex_locker.h>
#include <utils/system/fam.h>
#include <navgraph/navgraph.h>
#include <navgraph/yaml_navgraph.h>

#include <fawkes_msgs/NavGraph.h>
#include <fawkes_msgs/NavGraphGetPairwiseCosts.h>
#include <fawkes_msgs/NavGraphSearchPath.h>

#include <boost/filesystem.hpp>

#include <tf/transform_listener.h>

#define GET_CONFIG(privn, n, path, var)	  \
	if (! privn.getParam(path, var)) {      \
		n.getParam(path, var);                \
	}


class TransformListenerExtended : public tf::TransformListener
{
 public:
	TransformListenerExtended() : TransformListener() {}

	bool transform_origin(const std::string& source_frame,
	                      const std::string& target_frame,
	                      tf::Stamped<tf::Pose>& stamped_out)
	{
		tf::Stamped<tf::Pose> ident(tf::Transform(tf::Quaternion(0, 0, 0, 1),
		                                          tf::Vector3(0, 0, 0)),
		                            ros::Time(0,0), source_frame);

		transformPose(target_frame, ident, stamped_out);
		return true;
	}
};

class NavGraphRosNode : public fawkes::FamListener
{
 public:
	NavGraphRosNode(ros::NodeHandle &n)
		: n(n)
	{
		ros::NodeHandle privn("~");
		GET_CONFIG(privn, n, "navgraph_file", cfg_navgraph_file_);
		GET_CONFIG(privn, n, "base_frame_id", cfg_base_frame_);
		GET_CONFIG(privn, n, "global_frame_id", cfg_global_frame_);

		boost::filesystem::path p(cfg_navgraph_file_);
		p = boost::filesystem::absolute(p);
		cfg_navgraph_file_ = p.string();
		if (boost::filesystem::exists(p)) {
			navgraph = fawkes::LockPtr<fawkes::NavGraph>(fawkes::load_yaml_navgraph(cfg_navgraph_file_),
			                                             /* recursive mutex */ true);
			ROS_INFO("[F-NavGraph] Loaded navgraph from '%s': %zu nodes, %zu edges",
			         cfg_navgraph_file_.c_str(), navgraph->nodes().size(), navgraph->edges().size());
		} else {
			navgraph = fawkes::LockPtr<fawkes::NavGraph>(new fawkes::NavGraph("empty"), /* recursive mutex */ true);
		}

		boost::filesystem::create_directories(p.parent_path());
		fam_ = new fawkes::FileAlterationMonitor();
		fam_->add_filter((std::string("^") + p.filename().string() + "$").c_str());
		printf("Watching dir %s\n", p.parent_path().string().c_str());
		fam_->watch_dir(p.parent_path().string().c_str());
		//printf("Watching file %s\n", cfg_navgraph_file_.c_str());
		//fam_->watch_file(cfg_navgraph_file_.c_str());
		fam_->add_listener(this);

		tf_listener = new TransformListenerExtended();
		
		pub_navgraph_ = n.advertise<fawkes_msgs::NavGraph>("navgraph", 10, /* latching */ true);
		svs_search_path_ = n.advertiseService("navgraph/search_path",
		                                      &NavGraphRosNode::svs_search_path_cb, this);
		svs_get_pwcosts_ = n.advertiseService("navgraph/get_pairwise_costs",
		                                      &NavGraphRosNode::svs_get_pwcosts_cb, this);

		fam_timer_ =
			n.createWallTimer(ros::WallDuration(1.0), &NavGraphRosNode::cb_fam_timer, this);

		publish_graph();
}

	virtual ~NavGraphRosNode()
	{
		delete fam_;
		delete tf_listener;
	}

	virtual void
	fam_event(const char *filename, unsigned int mask)
	{
		// The file will be ignored from now onwards, re-register
		// if (mask & FAM_IGNORED) {
		// 	boost::filesystem::path p(cfg_navgraph_file_);
		// 	fam_->watch_dir(p.parent_path().string().c_str());
		// }

		if (mask & FAM_DELETE) {
			ROS_INFO("[F-NavGraph] Navgraph file deleted, clearing");
			navgraph->clear();
			publish_graph();
			return;
		}

		if (mask & (FAM_MODIFY | FAM_IGNORED)) {
			ROS_INFO("[F-NavGraph] NavGraph changed on disk, reloading");

			try {
				fawkes::LockPtr<fawkes::NavGraph> new_graph =
					fawkes::LockPtr<fawkes::NavGraph>(fawkes::load_yaml_navgraph(cfg_navgraph_file_),
					                                  /* recursive mutex */ true);
				ROS_INFO("[F-NavGraph] Re-loaded navgraph from '%s': %zu nodes, %zu edges",
				         cfg_navgraph_file_.c_str(), new_graph->nodes().size(), new_graph->edges().size());
				**navgraph = **new_graph;
				publish_graph();
			} catch (fawkes::Exception &e) {
				ROS_WARN("[F-NavGraph] Loading new graph failed: %s", e.what());
				return;
			} catch (std::runtime_error &e) {
				ROS_WARN("[F-NavGraph] Loading new graph failed: %s", e.what());
				return;
			}
		}
	}

	void
	cb_fam_timer(const ros::WallTimerEvent& event)
	{
		fam_->process_events();
	}

	void
	convert_nodes(const std::vector<fawkes::NavGraphNode> &nodes,
	                               std::vector<fawkes_msgs::NavGraphNode> &out)
	{
		for (const fawkes::NavGraphNode &node : nodes) {
			fawkes_msgs::NavGraphNode ngn;
			ngn.name = node.name();
			ngn.has_orientation = node.has_property(fawkes::navgraph::PROP_ORIENTATION);
			ngn.pose.x = node.x();
			ngn.pose.y = node.y();
			if (ngn.has_orientation) {
				ngn.pose.theta = node.property_as_float(fawkes::navgraph::PROP_ORIENTATION);
			}
			ngn.unconnected = node.unconnected();
			const std::map<std::string, std::string> &props = node.properties();
			for (const auto p : props) {
				fawkes_msgs::NavGraphProperty ngp;
				ngp.key = p.first;
				ngp.value = p.second;
				ngn.properties.push_back(ngp);
			}
			out.push_back(ngn);
		}
	}

	void
	publish_graph()
	{
		fawkes::MutexLocker lock(navgraph.objmutex_ptr());

		fawkes_msgs::NavGraph ngm;

		const std::vector<fawkes::NavGraphNode> &nodes = navgraph->nodes();
		convert_nodes(nodes, ngm.nodes);

		const std::vector<fawkes::NavGraphEdge> &edges = navgraph->edges();
		for (const fawkes::NavGraphEdge &edge : edges) {
			fawkes_msgs::NavGraphEdge nge;
			nge.from_node = edge.from();
			nge.to_node = edge.to();
			nge.directed = edge.is_directed();
			const std::map<std::string, std::string> &props = edge.properties();
			for (const auto p : props) {
				fawkes_msgs::NavGraphProperty ngp;
				ngp.key = p.first;
				ngp.value = p.second;
				nge.properties.push_back(ngp);
			}
			ngm.edges.push_back(nge);
		}
	
		pub_navgraph_.publish(ngm);
	}


	bool
	svs_search_path_cb(fawkes_msgs::NavGraphSearchPath::Request  &req,
	                   fawkes_msgs::NavGraphSearchPath::Response &res)
	{
		fawkes::NavGraphNode from, to;


		if (req.from_node.empty()) {
			tf::Stamped<tf::Pose> pose;
			if (! tf_listener->transform_origin(cfg_base_frame_, cfg_global_frame_, pose)) {
				ROS_WARN("[F-NavGraph] Failed to compute pose, cannot generate plan");

				res.ok = false;
				res.errmsg = "Failed to compute pose, cannot generate plan";
				return true;
			}

			from =
				navgraph->closest_node(pose.getOrigin().x(), pose.getOrigin().y());
			if (! from.is_valid()) {
				res.ok = false;
				res.errmsg = "Failed to get closest node to pose";
				return true;
			}

			fawkes_msgs::NavGraphNode free_start;
			free_start.name = "free-start";
			free_start.pose.x = pose.getOrigin().x();
			free_start.pose.y = pose.getOrigin().y();
			free_start.has_orientation = true;
			free_start.pose.theta = tf::getYaw(pose.getRotation());
			res.path.push_back(free_start);
		} else {
			from = navgraph->node(req.from_node);
			if (! from.is_valid()) {
				res.ok = false;
				res.errmsg = "Failed to find start node " + req.from_node;
				return true;
			}
		}

		fawkes::NavGraphPath path;

		if (! req.to_node.empty()) {
			path = navgraph->search_path(from.name(), req.to_node);
		} else {
			fawkes::NavGraphNode close_to_goal = navgraph->closest_node(req.to_pose.x, req.to_pose.y);
			path = navgraph->search_path(from, close_to_goal);
			if (! path.empty()) {
				fawkes::NavGraphNode free_target("free-target", req.to_pose.x, req.to_pose.y);
				if (std::isfinite(req.to_pose.theta)) {
					free_target.set_property("orientation", (float)req.to_pose.theta);
				}
				path.add_node(free_target, navgraph->cost(path.nodes().back(), free_target));
			}
		}

		// translate path into result
		convert_nodes(path.nodes(), res.path);
		res.cost = path.cost();
	
		res.ok = true;
		return true;
	}

	bool
	svs_get_pwcosts_cb(fawkes_msgs::NavGraphGetPairwiseCosts::Request  &req,
	                                    fawkes_msgs::NavGraphGetPairwiseCosts::Response &res)
	{
		for (unsigned int i = 0; i < req.nodes.size(); ++i) {
			for (unsigned int j = 0; j < req.nodes.size(); ++j) {
				if (i == j) continue;

				fawkes::NavGraphNode from_node, to_node;
				try {
					from_node = navgraph->node(req.nodes[i]);
					to_node = navgraph->node(req.nodes[j]);
				} catch (fawkes::Exception &e) {
					res.ok = false;
					res.errmsg = "Failed to get path from '" + req.nodes[i] + "' to '" +
						req.nodes[j] + "': " + e.what_no_backtrace();
					res.path_costs.clear();
					return true;					
				}

				fawkes::NavGraphNode start_node, goal_node;
				
				if (from_node.unconnected()) {
					start_node = navgraph->closest_node_to(from_node.name());
					//ROS_WARN("[F-NavGraph] From node %s is UNCONNECTED, starting instead from %s",
					//         from_node.name().c_str(), start_node.name().c_str());
				} else {
					start_node = from_node;
				}
				if (to_node.unconnected()) {
					goal_node = navgraph->closest_node_to(to_node.name());
					//ROS_WARN("[F-NavGraph] To node %s is UNCONNECTED, ending instead at %s",
					//         to_node.name().c_str(), goal_node.name().c_str());
				} else {
					goal_node = to_node;
				}
				fawkes::NavGraphPath p = navgraph->search_path(start_node, goal_node);
				if (p.empty()) {
					res.ok = false;
					res.errmsg = "Failed to get path from '" + start_node.name() + "' to '" + goal_node.name() + "'";
					res.path_costs.clear();
					return true;
				}
				fawkes_msgs::NavGraphPathCost pc;
				pc.from_node = req.nodes[i];
				pc.to_node = req.nodes[j];
				pc.cost = p.cost();
				if (from_node.unconnected()) {
					pc.cost += navgraph->cost(from_node, start_node);
				}
				if (to_node.unconnected()) {
					pc.cost += navgraph->cost(goal_node, to_node);
				}
				res.path_costs.push_back(pc);
			}
		}

		res.ok = true;
		return true;	
	}

	
 private:
	ros::NodeHandle    n;

	std::string cfg_navgraph_file_;
	std::string cfg_base_frame_;
  std::string cfg_global_frame_;

	fawkes::LockPtr<fawkes::NavGraph> navgraph;
	fawkes::FileAlterationMonitor *fam_;

  ros::Publisher pub_navgraph_;
  ros::ServiceServer svs_search_path_;
  ros::ServiceServer svs_get_pwcosts_;
	ros::WallTimer     fam_timer_;

	TransformListenerExtended *tf_listener;
};

int
main(int argc, char **argv)
{
	ros::init(argc, argv, "navgraph_node");

	ros::NodeHandle n;
	NavGraphRosNode navgraph_node(n);
  ros::spin();
  
	return 0;
}
