/***************************************************************************
 *  navgraph_retriever.cpp - retrieve navgraph and write to file
 *
 *  Created: Mon May 15 13:39:21 2017
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
#include <navgraph/navgraph.h>
#include <navgraph/navgraph_node.h>
#include <navgraph/navgraph_edge.h>
#include <navgraph/yaml_navgraph.h>

#include <fawkes_msgs/NavGraph.h>

#include <boost/filesystem.hpp>

#define GET_CONFIG(privn, n, path, var)	  \
	if (! privn.getParam(path, var)) {      \
		n.getParam(path, var);                \
	}

class NavGraphRosRetriever
{
 public:
	NavGraphRosRetriever(ros::NodeHandle &n)
		: n(n)
	{
		ros::NodeHandle privn("~");
		GET_CONFIG(privn, n, "navgraph_file", cfg_navgraph_file_);

		if (cfg_navgraph_file_.empty()) {
			throw std::runtime_error("No navgraph file given");
		}
		
		boost::filesystem::path p(cfg_navgraph_file_);
		p = boost::filesystem::absolute(p);
		cfg_navgraph_file_ = p.string();
		ROS_INFO("Using navgraph path: %s", p.string().c_str());
		if (boost::filesystem::exists(p)) {
			boost::filesystem::remove(p);
		}

		sub_navgraph_ = n.subscribe<fawkes_msgs::NavGraph>("navgraph", 10, &NavGraphRosRetriever::cb_navgraph, this);
	}

	virtual ~NavGraphRosRetriever()
	{
		sub_navgraph_.shutdown();
		boost::filesystem::path p(cfg_navgraph_file_);
		if (boost::filesystem::exists(p)) {
			boost::filesystem::remove(p);
		}
	}

	void
	cb_navgraph(const fawkes_msgs::NavGraph::ConstPtr& msg)
	{
		fawkes::NavGraph n("received");
		for (size_t i = 0; i < msg->nodes.size(); ++i) {
			const fawkes_msgs::NavGraphNode &msg_node = msg->nodes[i];
			//ROS_INFO("Adding Node %s", msg_node.name.c_str());
			std::map<std::string, std::string> properties;
			for (size_t j = 0; j < msg_node.properties.size(); ++j) {
				properties[msg_node.properties[j].key] = msg_node.properties[j].value;
			}
			fawkes::NavGraphNode node(msg_node.name, msg_node.pose.x, msg_node.pose.y, properties);
			node.set_unconnected(msg_node.unconnected);
			// has_orientation flag irrelevant, just means props have orientation field
			n.add_node(node);
		}
		
		for (size_t i = 0; i < msg->edges.size(); ++i) {
			const fawkes_msgs::NavGraphEdge &msg_edge = msg->edges[i];
			std::map<std::string, std::string> properties;
			for (size_t j = 0; j < msg_edge.properties.size(); ++j) {
				properties[msg_edge.properties[j].key] = msg_edge.properties[j].value;
			}
			//ROS_INFO("Adding Edge %s %s %s", msg_edge.from_node.c_str(),
			//         msg_edge.directed ? "->" : "--",
			//         msg_edge.to_node.c_str());
			fawkes::NavGraphEdge edge(msg_edge.from_node, msg_edge.to_node,
			                          properties, msg_edge.directed);
			n.add_edge(edge);
		}
		n.calc_reachability(/* allow multigraph */ true);
		ROS_INFO("Writing YAML navgraph '%s'", cfg_navgraph_file_.c_str());
		fawkes::save_yaml_navgraph(cfg_navgraph_file_, &n);
	}

 private:
	ros::NodeHandle    n;

	std::string cfg_navgraph_file_;

  ros::Subscriber sub_navgraph_;
};

int
main(int argc, char **argv)
{
	ros::init(argc, argv, "navgraph_retriever");

	ros::NodeHandle n;
	NavGraphRosRetriever navgraph_retriever(n);
  ros::spin();

	return 0;
}
