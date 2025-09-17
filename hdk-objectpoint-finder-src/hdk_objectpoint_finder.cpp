/*
 * @file hdk_objectpoint_finder.cpp
 * @brief ロボットの近くにあるobjectpointの名前を返すプログラム．
 *        ロボットの自己位置を受け取り，objectpoint名を返す．
 *        ROSサービスで実装．
 * @author Kosei Shino
 * @date 2024/7/22
 */

#include <cstdio>
#include <cstdlib>

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#include <cmath>
#undef _USE_MATH_DEFINES
#endif


#include "ros/ros.h"

#include "gnd_msgs/msg_pose2d_stamped.h"
#include "gnd/gnd_rosmsg_reader.hpp"

#include "std_msgs/String.h"

#include "gnd/gnd-path.hpp"
#include "gnd/gnd-path-io.hpp"
#include "gnd_msgs/msg_path_area_and_speed_limited.h"

#include "hdk/hdk_objectpoint_finder.hpp"

#include "gnd_msgs/msg_waypoint_named.h"

typedef gnd_msgs::msg_path_area_and_speed_limited				path_t;
typedef gnd::path::path_net_area_and_speed_limited				path_net_t;

typedef gnd_msgs::msg_pose2d_stamped 							msg_pose_t;
typedef gnd::rosutil::rosmsgs_reader_stamped<msg_pose_t>		msgreader_pose_t;

typedef hdk_objectpoint_finder::srv_find_objectpoint					srv_find_objectpoint_t;
typedef hdk::objectpoint_finder::srv_funcobj_find_objectpoint			srv_funcobj_find_objectpoint_t;
typedef hdk_objectpoint_finder::srv_is_in_travelable_area			srv_is_in_travelable_area_t;
typedef hdk::objectpoint_finder::srv_funcobj_is_in_travelable_area	srv_funcobj_is_in_travelable_area_t;


int main(int argc, char *argv[]) {
	hdk::objectpoint_finder::node_config node_config;

	{ // ---> start up, read configuration file
		if (argc > 1) {
			if (hdk::objectpoint_finder::fread_node_config(argv[1], &node_config) < 0) {
				char fname[1024];
				fprintf(stdout, "   ... Error: fail to read config file \"%s\"\n", argv[1]);
				sprintf(fname, "%s.tmp", argv[1]);
				// file out configuration file
				if (hdk::objectpoint_finder::fwrite_node_config(fname, &node_config) >= 0) {
					fprintf(stdout, "            : output sample configuration file \"%s\"\n", fname);
				}
				return -1;
			} else {
				fprintf(stdout, "   ... read config file \"%s\"\n", argv[1]);
			}
		}
	} // <--- start up, read configuration file

	{ // ---> initialize ros
		std::printf("\n");
		std::printf(" => initialize ros\n");
		if (node_config.node_name.value[0]) {
			ros::init(argc, argv, node_config.node_name.value);
			if( ros::isInitialized() ) {
				// nothing to do
				std::printf("    ... ok\n");
			} else {
				fprintf(stdout, "   ... Error: fail ROS initialization\n");
				return -1;
			}
		} else {
			fprintf(stdout, "   ... Error: node name is null, you must specify the name of this node via config item \"%s\"\n", node_config.node_name.item);
			return -1;
		}
	} // <--- initialize ros


	// ---> path object
	path_t path;
	path_net_t path_net;
	// <--- path object

	// ---> load route data file
	if( ros::ok() && node_config.path_map_file.value[0]) {
		fprintf(stderr, "\n");
		//fprintf(stderr, " => %d. load route data file\n", ++phase);
		fprintf(stderr, " => load route data file\n");
		fprintf(stderr, "    file path is \"%s\"\n", node_config.path_map_file.value);

		if( gnd::path::fread(node_config.path_map_file.value, &path_net) < 0 ) {
			fprintf(stderr, "    ... error : fail to read route file\n");
			ros::shutdown();
		}
		else {
			fprintf(stderr, "    ... ok\n");
		}
	} // <--- load route data file


	// ---> ros communication object
	ros::NodeHandle			nh;

	/* pose subscriber */
	ros::Subscriber						subsc_pose;
	msgreader_pose_t					msgreader_pose;
	msg_pose_t							msg_pose;
	/* find objectpoint server */
	ros::ServiceServer					srvserv_find_objectpoint;
	srv_funcobj_find_objectpoint_t			sevfo_find_objectpoint(path_net);
	/* is in travelable area server */
	ros::ServiceServer					srvserv_is_in_travelable_area;
	srv_funcobj_is_in_travelable_area_t	sevfo_is_in_travelable_area(path_net);
	// <--- ros communication object

	// ---> make pose subscriber
	if (ros::ok()) {
		fprintf(stdout, "\n");
		fprintf(stdout, "   => make pose subscriber\n");

		if( !node_config.topic_name_pose.value ) {
			fprintf(stderr, "    ... error: pose topic name is null\n");
			fprintf(stderr, "        usage: fill \"%s\" item in configuration file\n", node_config.topic_name_pose.item );
			ros::shutdown();
		}
		else {
			fprintf(stdout, "    ... topic name is \"%s\"\n", node_config.topic_name_pose.value);

			// allocate buffer
			msgreader_pose.allocate(100);

			// subscribe
			subsc_pose = nh.subscribe(node_config.topic_name_pose.value, 100,
					&msgreader_pose_t::rosmsg_read,
					msgreader_pose.reader_pointer() );

			msg_pose.header.seq = 0;
			msg_pose.header.stamp.fromSec(0.0);
			msg_pose.header.frame_id = "";

			fprintf(stdout, "    ... ok\n");
		}
	} // <--- make pose subscriber


	{ // ---> make service find objectpoint server
		fprintf(stdout, "\n");
		fprintf(stdout, " => make service server to find objectpoint\n");

		if( !node_config.service_name_find_objectpoint.value[0] ) {
			std::printf("    ... error: invalid service name\n");
				ros::shutdown();
		} else {
			std::printf("    ... service name is \"%s\"\n", node_config.service_name_find_objectpoint.value );
			srvserv_find_objectpoint = nh.advertiseService(
					node_config.service_name_find_objectpoint.value,
					&srv_funcobj_find_objectpoint_t::callback, &sevfo_find_objectpoint);
		}
	} // <--- make service find objectpoint server


	{ // ---> make service is in travelable area server
		fprintf(stdout, "\n");
		fprintf(stdout, " => make service server to check specified position is in travelable area\n");

		if( !node_config.service_name_is_in_travelable_area.value[0] ) {
			std::printf("    ... error: invalid service name\n");
				ros::shutdown();
		} else {
			std::printf("    ... service name is \"%s\"\n", node_config.service_name_is_in_travelable_area.value );
			srvserv_is_in_travelable_area = nh.advertiseService(
					node_config.service_name_is_in_travelable_area.value,
					&srv_funcobj_is_in_travelable_area_t::callback, &sevfo_is_in_travelable_area);
		}
	} // <--- make service is in travelable area server


	// ---> operation
	if ( ros::ok() ) {
	//	ros::Rate loop_rate(100.0);
	ros::Rate loop_rate(10);
		/* time */
		double time_start;
		double time_current;
		double time_display_status;
		double time_display_status_prev;
		double time_display_image;
		double time_display_image_prev;
		time_start = floor(ros::Time::now().toSec());
		time_current = time_start;
		time_display_status = time_start;
		time_display_status_prev = time_start;
		time_display_image = time_start;
		time_display_image_prev = time_start;

		std::size_t nline_display = 0; // for display status

    	//ros::Publisher pub_closest_objectpoint = nh.advertise<hdk_objectpoint_finder::ObjectPointInfo>("closest_objectpoint", 10);
		ros::Publisher pub_closest_objectpoint = nh.advertise<gnd_msgs::msg_waypoint_named>("closest_objectpoint", 10);
		
		// ---> main loop
		std::printf("\n");
		std::printf(" => main loop start\n");
		while ( ros::ok() ) {
			/* blocking */
			loop_rate.sleep();

			/* spin */
			ros::spinOnce();

			/* save time */
			time_current = ros::Time::now().toSec();

			/* update pose */
			if( (msgreader_pose.copy_latest( &msg_pose ) == 0) ) {
				hdk::objectpoint_finder::pose2d_t pose;
				pose.x = msg_pose.x;
				pose.y = msg_pose.y;
				pose.theta = msg_pose.theta;

				sevfo_find_objectpoint.set_pose(pose);
				sevfo_is_in_travelable_area.set_pose(pose);

				// 最も近いobjectpointを見つけてパブリッシュ
				char closest_objectpoint[128];
				if (hdk::objectpoint_finder::find_objectpoint(path_net, pose.x, pose.y, 0.0, "", closest_objectpoint)) {
			//		hdk_objectpoint_finder::ObjectPointInfo msg;
			    	gnd_msgs::msg_waypoint_named msg;
					msg.name = closest_objectpoint;
					
					double x, y;
					if (path_net.get_objectpoint(closest_objectpoint, &x, &y)) {
						msg.x = x;
						msg.y = y;
						pub_closest_objectpoint.publish(msg);
						ROS_INFO("Closest objectpoint: %s (%.2f, %.2f)", msg.name.c_str(), msg.x, msg.y);
					} else {
						ROS_WARN("Found objectpoint but couldn't get its position: %s", closest_objectpoint);
					}
				} else {
						ROS_INFO("No objectpoint found");
					}

			}

			// ---> display status
			if ( node_config.period_cui_status_display.value
					&& time_current > time_display_status) {

				if (nline_display) {
					std::printf("\x1b[%02dA", nline_display);
					nline_display = 0;

				}
			//	bool is_in_travelable_area = false;
			//	is_in_travelable_area = hdk::objectpoint_finder::is_in_path_net( path_net, msg_pose.x, msg_pose.y, 0.0 );


				nline_display++;
				std::printf("\x1b[K-------------------- \x1b[1m\x1b[36m%s\x1b[39m\x1b[0m --------------------\n",
						node_config.node_name.value);

				nline_display++; std::printf("\x1b[K                               operating time : %6.01lf[sec]\n", time_current - time_start);
				nline_display++; std::printf("\x1b[K                callback count(find_objectpoint) : %6d[times]\n", sevfo_find_objectpoint.get_count_callback());
			//	nline_display++; std::printf("\x1b[K                        is_in_travelable_area : %s\n", is_in_travelable_area? "true": "false" );

				// update
				time_display_status_prev = time_display_status;
				time_display_status = gnd_loop_next(time_current, time_start, node_config.period_cui_status_display.value );
			} // <--- display status


		} // <--- main loop
	} // <--- operation

	return 0;
}
