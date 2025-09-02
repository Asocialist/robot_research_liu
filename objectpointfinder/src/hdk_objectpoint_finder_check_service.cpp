/*
 * @file hdk_objectpoint_finder_check_service.cpp
 * @brief hdk_objectpoint_finderのサービスをコールするプログラム．
 *
 * 引数はコマンドラインで与える．
 *
 * @author Hidekazu TAKAHASHI
 * @date 2018/01/10
 */
#include <cstddef>
#include <cstdio>
#include <cstdlib>

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#include <cmath>
#undef _USE_MATH_DEFINES
#endif


#include "ros/ros.h"

#include "hdk_objectpoint_finder/srv_find_objectpoint.h"

const char *node_name = "hdk_objectpoint_finder_check_service";
const char *service_name = "find_objectpoint";

typedef hdk_objectpoint_finder::srv_find_objectpoint srv_find_objectpoint_t;
typedef hdk_objectpoint_finder::srv_find_objectpointRequest request_t;
typedef hdk_objectpoint_finder::srv_find_objectpointResponse response_t;

int main(int argc, char *argv[]) {
	if ( argc != 2 ) {
		std::fprintf(stderr, "Usage: $program dst\n");
		std::exit(EXIT_FAILURE);
	}

	{ // ---> init ros
		ros::init(argc, argv, node_name);
		if( ros::isInitialized() ) {
			// nothing to do
		} else {
			std::fprintf(stdout, "[Error]: fail ROS initialization\n");
			std::fprintf(stdout, "exit\n");
			ros::shutdown();
			std::exit(EXIT_FAILURE);
		}
	} // <--- init ros


	// ---> ros communication object
	ros::NodeHandle node_handle;
	ros::ServiceClient client = node_handle.serviceClient<srv_find_objectpoint_t>(service_name);
	srv_find_objectpoint_t srv;
	// <--- ros communication object

	// ---> call service
	if ( ros::ok() ) {
		srv.request.objectpoint_name_destination = std::string(argv[1]);
		if ( client.call(srv) ) {
			std::fprintf(stdout, "objectpoint_name: \"%s\"\n", srv.response.objectpoint_name.c_str());
		} else {
			std::fprintf(stderr, "[Error]: service call returns false\n");
			std::fprintf(stderr, "         do you start hdk_objectpoint_finder?\n");
			std::fprintf(stderr, "         is your robot in travelable area?\n");
			std::fprintf(stderr, "         does the destination specified exist?\n");
		}
	} // <--- call service


	return 0;
}
