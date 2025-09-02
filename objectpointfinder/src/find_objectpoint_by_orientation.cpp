/*
 * @file find_objectpoint_by_orientation.cpp
 * @brief 人の向いている方向にあるobjectpointの名前を返すプログラム．
 *        ロボットの自己位置、人の位置・向きを受け取り，objectpoint名を返す．
 *        ROSサービスで実装．
 * @author Kosei SHINO
 * @date 2024/9/30
 * 11/26
 */

#include <cstdio>
#include <cstdlib>

#include <stdio.h>

#define ROOT 0.87

#include <string>
#include "json11.hpp"
#include "DataTypedef.h"

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#include <cmath>
#undef _USE_MATH_DEFINES
#endif

#include "ros/ros.h"

#include "gnd_msgs/msg_pose2d_stamped.h"
#include "gnd/gnd_rosmsg_reader.hpp"

#include "geometry_msgs/Pose2D.h"

#include "std_msgs/String.h"

#include "gnd/gnd-path.hpp"
#include "gnd/gnd-path-io.hpp"
#include "gnd_msgs/msg_path_area_and_speed_limited.h"

#include "hdk/find_objectpoint_by_orientation.hpp"

#include "gnd_msgs/msg_waypoint_named.h"

#include "gnd/gnd-matrix-base.hpp"
#include "gnd/gnd-vector-base.hpp"

#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"

typedef gnd_msgs::msg_path_area_and_speed_limited				path_t;
typedef gnd::path::path_net_area_and_speed_limited				path_net_t;

typedef gnd_msgs::msg_pose2d_stamped 							msg_pose_t;
typedef gnd::rosutil::rosmsgs_reader_stamped<msg_pose_t>		msgreader_pose_t;

typedef hdk_objectpoint_finder::srv_find_objectpoint					srv_find_objectpoint_t;
typedef hdk::objectpoint_finder::srv_funcobj_find_objectpoint			srv_funcobj_find_objectpoint_t;
typedef hdk_objectpoint_finder::srv_is_in_travelable_area			srv_is_in_travelable_area_t;
typedef hdk::objectpoint_finder::srv_funcobj_is_in_travelable_area	srv_funcobj_is_in_travelable_area_t;

std::string defTopicNameTrackingPosition = "pose_person_following";
std::string defTopicNameVehiclePose = "pose_particle_localizer";
std::string defPathFileName = "/home/liu/liu_workplace/ros/locations/250725/250725.path";
//std::string defPathFileName = "/home/kobayashilab/ros/locations/bldg_RandP_5F/bldg_RandP_5F.path";
//std::string defPathFileName = "/home/kobayashilab/ros/locations/shino_5f/shino_5f.path";

// グローバル変数
Vec2d_t poseTracking = Vec2d_t{0.0, 0.0};

double poseTrackingT = 0;
bool isReceivedLatestTracking = false;
Vec2d_t poseVehicle = Vec2d_t{0.0, 0.0};
double poseVehicleT = 0;
bool defIsTrackingPositionGlobal = false; // この値は適切に設定してください

namespace hdk {
namespace objectpoint_finder {

// Vec2d_tの長さを計算する関数
double norm(const Vec2d_t& vec) {
    return std::sqrt(vec.x * vec.x + vec.y * vec.y);
}

double inner_prod(const Vec2d_t &a, const Vec2d_t &b){
	return a.x*b.x+a.y*b.y;
}

// // 角度を-π〜πの範囲に正規化する関数
// double angle_normalize(double angle) {
//     while (angle > M_PI) angle -= 2 * M_PI;
//     while (angle < -M_PI) angle += 2 * M_PI;
//     return angle;
// }

bool find_objectpoint_in_direction(gnd::path::path_net_area_and_speed_limited &path_net,
    const Vec2d_t pos, double theta, double max_distance, double angle_threshold,
    const char *objectpoint_name_destination, char *objectpoint_name) {
    
    ROS_INFO("Person's current position: (%.2f, %.2f), orientation: %.2f rad (%.2f deg)", 
             pos.x, pos.y, theta, theta * 180.0 / M_PI);

    // Validate input
    if (std::isnan(pos.x) || std::isnan(pos.y) || std::isnan(theta)) {
        ROS_WARN("Invalid input: position or orientation contains NaN");
        return false;
    }

    // First, try to find an object point in the user's direction
    std::size_t i;
    double min_direction_distance = std::numeric_limits<double>::max();
    int closest_direction_index = -1;
    char current_name[128];

    // First pass: Find object point in the specified direction
    for (i = 0; i < path_net.n_objectpoints(); i++) {
        path_net.name_objectpoint(i, current_name);
        
        double obj_x, obj_y;
        path_net.get_objectpoint(current_name, &obj_x, &obj_y);
        
        Vec2d_t obj_pos(obj_x, obj_y);
        Vec2d_t diff = obj_pos - pos;
        double distance = norm(diff);
        //double angle = std::atan2(diff.y, diff.x);
        
        // Calculate angle difference
        //double angle_diff = std::abs(angle_normalize(angle - theta));

        // calculate angle difference between "companion's viewing direction" and "object's direction from companion" by inner product
        Vec2d_t ndiff = (diff/distance);
        Vec2d_t personDir(cos(theta),sin(theta));
        double cvd = inner_prod(personDir, ndiff);
        double angle_diff = acos(cvd); //[0, pi]


        // Check if object point is within search criteria
        if (distance <= max_distance && 
            angle_diff <= angle_threshold && 
            distance < min_direction_distance) {
            min_direction_distance = distance;
            closest_direction_index = i;
            
            ROS_INFO("Directional point found: %s (dist=%.2f, angle_diff=%.2f deg)", 
                      current_name, distance, angle_diff * 180.0 / M_PI);
        }
    }

    // If an object point in the direction is found, return it
    if (closest_direction_index != -1) {
        path_net.name_objectpoint(closest_direction_index, objectpoint_name);
        ROS_INFO(GREEN "Found directional objectpoint: %s at distance %.2f" RESET, 
                 objectpoint_name, min_direction_distance);
        return true;
    }

  //位置で補正する際はここのコメントアウト外す
    // If no object point found in the specified direction, 
    // fall back to finding the nearest object point
/*
	
    double min_overall_distance = std::numeric_limits<double>::max();
    int closest_overall_index = -1;

    for (i = 0; i < path_net.n_objectpoints(); i++) {
        path_net.name_objectpoint(i, current_name);
        
        double obj_x, obj_y;
        path_net.get_objectpoint(current_name, &obj_x, &obj_y);
        
		Vec2d_t obj_pos(obj_x, obj_y);
        Vec2d_t diff = obj_pos - pos;
        double distance = norm(diff);

        if (distance < min_overall_distance) {
            min_overall_distance = distance;
            closest_overall_index = i;
        }
    }

    // If a nearest object point is found
    if (closest_overall_index != -1) {
        path_net.name_objectpoint(closest_overall_index, objectpoint_name);
       // ROS_WARN(YELLOW "No directional objectpoint found. Using nearest point: %s at distance %.2f" RESET, 
        //         objectpoint_name, min_overall_distance);
        return true;
    }

*/

    // No object points found at all
    ROS_ERROR("No objectpoints found in the path network");
    return false;
}
} // namespace objectpoint_finder
} // namespace hdk

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

// 人物の位置情報サブスクライブ
    ros::Subscriber subPersonPosition = nh.subscribe(defTopicNameTrackingPosition.c_str(), 100,
        +[](const geometry_msgs::Pose2D::ConstPtr& msg) {
            if(defIsTrackingPositionGlobal){
                poseTracking.x = msg->x;
                poseTracking.y = msg->y;
                poseTrackingT = msg->theta;
                ROS_INFO("Received person global position: x=%.2f, y=%.2f, a=%.2f", msg->x, msg->y, poseTrackingT);
            }
            else{
                poseTracking.x = msg->x * cos(poseVehicleT) - msg->y * sin(poseVehicleT) + poseVehicle.x;
                poseTracking.y = msg->x * sin(poseVehicleT) + msg->y * cos(poseVehicleT) + poseVehicle.y;
                poseTrackingT = msg->theta + poseVehicleT;
                //ROS_INFO("Received person local position -> global: x=%.2f, y=%.2f, a=%.2f", poseTracking.x, poseTracking.y, poseTrackingT);
            }
            isReceivedLatestTracking = true;
            return;
        });

// ロボットの位置情報サブスクライブ
    ros::Subscriber subVehiclePosition = nh.subscribe(defTopicNameVehiclePose.c_str(), 100, 
    +[](const gnd_msgs::msg_pose2d_stamped::ConstPtr& msg) {     
			poseVehicle.x = msg->x;
			poseVehicle.y = msg->y; 
			poseVehicleT  = msg->theta;     
           // ROS_INFO("Received robot pose: x=%.2f, y=%.2f, theta=%.2f", msg->x, msg->y, msg->theta);

        return;
    });

    	//ros::Publisher pub_closest_objectpoint = nh.advertise<hdk_objectpoint_finder::ObjectPointInfo>("closest_objectpoint", 10);
		ros::Publisher pub_closest_objectpoint = nh.advertise<gnd_msgs::msg_waypoint_named>("closest_objectpoint", 10);
		
		// ---> main loop
		std::printf("\n");
		std::printf(" => main loop start\n");

	//	ros::Rate rate(1); // 1Hz
		
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
			}


		if (isReceivedLatestTracking) {
			// 探索パラメータを適切な値に調整　可変
			double max_search_distance = 2.0;  // より広い探索範囲（メートル）
			double angle_threshold = 25.0 * M_PI / 180.0;  // より広い角度範囲（35度）
		/*
            hdk::objectpoint_finder::pose2d_t pose;
            pose.x = poseTracking.x;
            pose.y = poseTracking.y;
            pose.theta = poseTrackingT;

            sevfo_find_objectpoint.set_pose(pose);
            sevfo_is_in_travelable_area.set_pose(pose);
		*/


            // 人物の向きに基づいてオブジェクトポイントを見つけてパブリッシュ
            char closest_objectpoint[128];
         /*   if (hdk::objectpoint_finder::find_objectpoint_in_direction(path_net, poseTracking, poseTrackingT, 
                                                                       10.0, 20 * M_PI / 180, "", closest_objectpoint)) {
        */
	    if (hdk::objectpoint_finder::find_objectpoint_in_direction(
            path_net, 
            poseTracking, 
            poseTrackingT, 
            max_search_distance,
            angle_threshold, 
            "", 
            closest_objectpoint)) {

		        gnd_msgs::msg_waypoint_named msg;
                msg.name = closest_objectpoint;
                
                double x, y;
                if (path_net.get_objectpoint(closest_objectpoint, &x, &y)) {
                    msg.x = x;
                    msg.y = y;
                    pub_closest_objectpoint.publish(msg);
					ROS_INFO_STREAM(GREEN << "publish!!!! " 
										<< msg.name.c_str() << RESET);
                } else {
                    ROS_WARN("Found objectpoint but couldn't get its position: %s", closest_objectpoint);
                }
            } else {
                ROS_ERROR("No objectpoint");
            }
			}

      //      isReceivedLatestTracking = false;
       // }

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
			//	std::printf("\x1b[K-------------------- \x1b[1m\x1b[36m%s\x1b[39m\x1b[0m --------------------\n",
			//			node_config.node_name.value);

			//	nline_display++; std::printf("\x1b[K                               operating time : %6.01lf[sec]\n", time_current - time_start);
			//	nline_display++; std::printf("\x1b[K                callback count(find_objectpoint) : %6d[times]\n", sevfo_find_objectpoint.get_count_callback());
			//	nline_display++; std::printf("\x1b[K                        is_in_travelable_area : %s\n", is_in_travelable_area? "true": "false" );

				// update
				time_display_status_prev = time_display_status;
				time_display_status = gnd_loop_next(time_current, time_start, node_config.period_cui_status_display.value );
			} // <--- display status

		//rate.sleep();
		
		} // <--- main loop
	} // <--- operation

	return 0;
}
