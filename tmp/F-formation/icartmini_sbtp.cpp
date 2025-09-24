/**
 * @file icartmini_sbtp/src/icartmini_sbtp.cpp
 *
 * @brief icartmini Selection-Based Trajectory Planning
 **/

#include "gnd/gnd-multi-platform.h"
#include "icartmini_sbtp/icartmini_sbtp.hpp"

#include "ros/ros.h"

//#include "jwvehicle/jwvehicle.hpp"
#include "gnd/gnd-util.h"
#include "gnd/gnd-vector-base.hpp"
#include "gnd/gnd-matrix-coordinate.hpp"
#include "gnd/gnd-coord-tree.hpp"

#include "gnd/gnd-path-io.hpp"

#include "gnd_msgs/msg_Fformation_mode.h"


#include <ypspur.h>

typedef icartmini_sbtp::path_t								path_t;
typedef icartmini_sbtp::path_net_t							path_net_t;

typedef icartmini_sbtp::node_config							node_config_t;
typedef icartmini_sbtp::msg_pose_t							msg_pose_t;
typedef icartmini_sbtp::msgreader_pose_t					msgreader_pose_t;
typedef icartmini_sbtp::msg_pointcloud_t					msg_pointcloud_t;
typedef icartmini_sbtp::msgreader_pointcloud_t				msgreader_pointcloud_t;
typedef icartmini_sbtp::msg_velocity_t						msg_velocity_t;
typedef icartmini_sbtp::msgreader_velocity_t				msgreader_velocity_t;
typedef icartmini_sbtp::msg_trajectory_t					msg_trajectory_t;
typedef icartmini_sbtp::msg_path_unit_t						msg_path_unit_t;
typedef icartmini_sbtp::msg_path_t							msg_path_t;
//typedef icartmini_sbtp::msg_vehicle_ctrl_t					msg_vehicle_ctrl_t;
typedef icartmini_sbtp::msg_vehicle_status_t				msg_vehicle_status_t;
typedef icartmini_sbtp::msg_mode_status_t					msg_mode_status_t;
typedef icartmini_sbtp::msgreader_mode_status_t				msgreader_mode_status_t;
typedef icartmini_sbtp::msg_controller_t					msg_controller_t;
typedef icartmini_sbtp::msgreader_controller_t				msgreader_controller_t;
typedef icartmini_sbtp::msg_pose_companion_t				msg_pose_companion_t;
typedef icartmini_sbtp::msgreader_pose_companion_t			msgreader_pose_companion_t;
typedef jwvehicle_sbtp::srv_set_navigation_path				srv_set_navigation_path_t;
typedef icartmini_sbtp::srv_funcobj_set_navigation_path_t	srv_funcobj_set_navigation_path_t;

const int IndexSquare = 0;
const int IndexCross = 1;
const int IndexCircle = 2;
const int IndexTriangle = 3;
const int IndexL1 = 4;
const int IndexR1 = 5;
const int IndexL2 = 6;
const int IndexR2 = 7;
const int IndexShare = 8;
const int IndexOptions = 9;
const int IndexL3 = 10;
const int IndexR3 = 11;
const int IndexHome = 12;
const int IndexTouchpad = 13;
const int LengthButtons = 14;

bool formation_mode = false;
double formation_x, formation_y, formation_angle, formation_linear_velocity;

void formationModeCallback(const gnd_msgs::msg_Fformation_mode::ConstPtr& msg)
{
	formation_mode = msg->formation_flag;
	//formation_x = msg->x;
	//formation_y = msg->y;
	formation_angle = msg->angle;
	formation_linear_velocity = msg->linear_velocity;
}

int main(int argc, char **argv) {
	node_config_t node_config;
	{ // ---> start up, read configuration file
		if( argc > 1 ) {
			if( icartmini_sbtp::fread_node_config( argv[1], &node_config ) < 0 ){
				char fname[1024];
				fprintf(stdout, "   ... error : fail to read configuration file \"%s\"\n", argv[1]);
				sprintf(fname, "%s.tmp", argv[1]);
				// file out configuration file
				if( icartmini_sbtp::fwrite_node_config( fname, &node_config ) >= 0 ){
					fprintf(stdout, "            : output sample configuration file \"%s\"\n", fname);
				}
				return -1;
			}
			else {
				fprintf(stdout, "   ... read configuration file \"%s\"\n", argv[1]);
			}
		}
		else {
			char fname[1024];
			fprintf(stdout, "   ... error : missing configuration file path operand\n");
			fprintf(stdout, "   ... usage : ./%s <config file>\n", argv[0]);

			sprintf(fname, "sample.conf");
			if( icartmini_sbtp::fwrite_node_config( "sample.conf", &node_config ) >= 0 ){
				fprintf(stdout, "            : output sample configuration file \"%s\"\n", fname);
			}
			return -1;
		}
	} // <--- start up, read configuration file

	{ // ---> initialize ros�ｿｽ@platform
		if( node_config.node_name.value[0] ) {
			ros::init(argc, argv, node_config.node_name.value);
		}
		else {
			fprintf(stdout, "   ... error: node name is null, you must specify the name of this node via config item \"%s\"\n", node_config.node_name.item);
			return -1;
		}
		fprintf(stdout, " node: \"%s\"\n", node_config.node_name.value);
	} // <--- initialize ros�ｿｽ@platform

	{ // ---> initialize ypspur
		if(	::Spur_init() < 0 ) {
			std::fprintf(stderr, "   ... error: failed to initialyze Spur\n");
			return -1;
		} else {
			;
		}
	} // <--- intialize ypspur


	// ---> ros communication object
	ros::NodeHandle						nodehandle;

	ros::Subscriber						subsc_pose;
	msg_pose_t							msg_pose;
	msgreader_pose_t					msgreader_pose;

	typedef struct  {
		ros::Subscriber						subsc_pointcloud;
		msg_pointcloud_t					msg;
		msgreader_pointcloud_t				msgreader;
	} topic_pointcloud_t;
	topic_pointcloud_t *topics_pointcloud;
	int ntopics_pointcloud = 0;

	ros::Subscriber						subsc_velocity;
	msg_velocity_t						msg_velocity;
	msgreader_velocity_t				msgreader_velocity;

	ros::Publisher						pub_planned_path;
	msg_path_t							msg_planned_path;

	//ros::Publisher						pub_vehicle_ctrl;
	//msg_vehicle_ctrl_t					msg_vehicle_ctrl;

	ros::Publisher						pub_vehicle_stauts;
	msg_vehicle_status_t				msg_vehicle_status;

	ros::Publisher						pub_trajectory_target;
	msg_trajectory_t					msg_trajectory_target;

	ros::Publisher						pub_trajectory_actual;
	msg_trajectory_t					msg_trajectory_actual;

	ros::Subscriber						subsc_controller;
	msg_controller_t					msg_controller;
	msg_controller_t					msg_controller_prev;
	msgreader_controller_t				msgreader_controller;

	ros::Subscriber						subsc_pose_companion;
	msg_pose_companion_t				msg_pose_companion;
	msgreader_pose_companion_t			msgreader_pose_companion;

	ros::Subscriber						subsc_mode_status;
	msg_mode_status_t					msg_mode_status;
	msgreader_mode_status_t				msgreader_mode_status;

	ros::Subscriber						subsc_external_controll_velocity;
	static double 						external_controlled_velocity;

	ros::ServiceServer					srvserver_set_navigation_path;
	srv_funcobj_set_navigation_path_t	srv_funcobj_set_navigation_path;
	// <--- ros communication object

	path_net_t path_net;

	{ // ---> initialize
		int phase = 0;
		ros::Time time_start;
		fprintf(stdout, "---------- initialize ----------\n");

		// ---> show initialization task
		if( ros::ok() ) {
			fprintf(stdout, " => show task\n");
			if( node_config.path_file.value[0] )
				fprintf(stdout, "   %d. load route data file\n", ++phase);
			if( node_config.path_file.value[0] && node_config.start_node.value[0] && node_config.dest_node.value[0] ) {
				fprintf(stderr, "    %d. find path\n", ++phase);
			}
			fprintf(stdout, "   %d. make pose subscriber for navigation\n", ++phase);
			if( node_config.topic_names_pointcloud.size() > 0 ) {
				fprintf(stdout, "   %d. make point-cloud subscriber for obstacle detection\n", ++phase);
			}
			fprintf(stdout, "   %d. make vehicle control command publisher\n", ++phase);
			if( node_config.topic_name_pose_companion.value[0] ) {
				fprintf(stdout, "   %d. make companion pose subscriber to accompany\n", ++phase);
			}
			fprintf(stdout, "   %d. make controller subscriber for obstacle detection\n", ++phase);
			fprintf(stdout, "\n");
		} // <--- show initialization task

		phase = 0;
		// ---> load route data file
		if( ros::ok() && node_config.path_file.value[0]) {
			fprintf(stderr, "\n");
			fprintf(stderr, " => %d. load route data file\n", ++phase);
			fprintf(stderr, "        file path is \"%s\"\n", node_config.path_file.value);

			if( gnd::path::fread(node_config.path_file.value, &path_net) < 0 ) {
				fprintf(stderr, "    ... error : fail to read route file\n");
				ros::shutdown();
			}
			else {
				fprintf(stderr, "    ... ok\n");
			}
		} // <--- load route data file

		// ---> search path
		if( ros::ok() && path_net.n_waypoints() > 0 &&
				node_config.start_node.value[0] && node_config.dest_node.value[0] ){
			path_t ws;
			fprintf(stderr, "\n");
			fprintf(stderr, " => %d. find path from \"%s\" to \"%s\"\n", ++phase, node_config.start_node.value, node_config.dest_node.value);

			// plan a path to destination
			if( path_net.find_path_dijkstra( &ws, node_config.start_node.value, node_config.dest_node.value ) < 0 ) {
				fprintf(stderr, "    ... error : fail to find path\n");
				ros::shutdown();
			}
			else {
				int i;

				fprintf(stdout, "path: %s", ws.start.name);
				msg_planned_path.start.x = ws.start.x;
				msg_planned_path.start.y = ws.start.y;
				msg_planned_path.start.theta = 0;
				msg_planned_path.start.name = ws.start.name;
				for( i = 0; i < (signed) ws.path.size(); i++ ){
					msg_path_unit_t unit;

					fprintf(stdout, " -> %s", ws.path[i].end.name );
					unit.end.x = ws.path[i].end.x;
					unit.end.y = ws.path[i].end.y;
					unit.end.theta = ws.path[i].end.theta;
					unit.end.name = ws.path[i].end.name;

					unit.curvature = ws.path[i].curvature;
					unit.limit_translate = ws.path[i].prop.limit_translate;
					unit.limit_rotate = ws.path[i].prop.limit_rotate;

					msg_planned_path.path.push_back(unit);
				}
				fprintf(stdout, "\n" );

				fprintf(stderr, "    ... ok\n");
			}
		} // <--- search path


		// ---> make pose subscriber for navigation
		if( ros::ok() ) {
			fprintf(stderr, "\n");
			fprintf(stdout, " =>  %d. make pose subscriber for navigation\n", ++phase);

			if( !node_config.topic_name_pose.value[0] ) {
				fprintf(stdout, "    ... error: invalid topic name\n");
				ros::shutdown();
			}
			else {
				fprintf(stdout, "    ... topic name is \"%s\"\n", node_config.topic_name_pose.value );

				// allocate buffer
				msgreader_pose.allocate(400);

				// subscribe
				subsc_pose = nodehandle.subscribe(node_config.topic_name_pose.value, 10,
						&msgreader_pose_t::rosmsg_read,
						msgreader_pose.reader_pointer() );

				fprintf(stdout, "    ... ok\n");
			}
		} // <--- make pose subscriber for navigation


		// ---> make point-cloud subscriber for obstacle detection
		if( ros::ok() && node_config.topic_names_pointcloud.size() > 0) {
			fprintf(stderr, "\n");
			fprintf(stdout, " => %d. make point-cloud subscriber for obstacle detection\n", ++phase);

			topics_pointcloud = new topic_pointcloud_t[node_config.topic_names_pointcloud.size()];
			if( !topics_pointcloud ) {
				fprintf(stdout, "    ... error: fail to allocate memories\n");
				ros::shutdown();
			}
			else {
				ntopics_pointcloud = node_config.topic_names_pointcloud.size();

				for( int i = 0; i < (signed)node_config.topic_names_pointcloud.size(); i++ ) {
					if( !node_config.topic_names_pointcloud[i].value[0] ) {
						fprintf(stdout, "    ... error: invalid topic name\n");
						ros::shutdown();
					}
					else {
						fprintf(stdout, "    ... topic name is \"%s\"\n", node_config.topic_names_pointcloud[i].value );

						// allocate buffer
						topics_pointcloud[i].msgreader.allocate(80);

						// subscribe
						topics_pointcloud[i].subsc_pointcloud = nodehandle.subscribe(node_config.topic_names_pointcloud[i].value, 10,
								&msgreader_pointcloud_t::rosmsg_read,
								topics_pointcloud[i].msgreader.reader_pointer() );

						fprintf(stdout, "    ... ok\n");
					}
				}
			}
		} // <--- make point-cloud subscriber for obstacle detection


		// ---> make vehicle velocity subscriber
		if( ros::ok() ) {
			fprintf(stderr, "\n");
			fprintf(stdout, " => %d. make vehicle velocity subscriber for obstacle detection\n", ++phase);

			if( !node_config.topic_name_vehicle_vel.value[0] ) {
				fprintf(stdout, "    ... error: invalid topic name\n");
				ros::shutdown();
			}
			else {
				fprintf(stdout, "    ... topic name is \"%s\"\n", node_config.topic_name_vehicle_vel.value );

				// allocate buffer
				msgreader_velocity.allocate(400);

				// subscribe
				subsc_velocity = nodehandle.subscribe(node_config.topic_name_vehicle_vel.value, 10,
						&msgreader_velocity_t::rosmsg_read,
						msgreader_velocity.reader_pointer() );

				fprintf(stdout, "    ... ok\n");
			}
		} // <--- make vehicle velocity subscriber

		// ---> initialize paths publisher
		if( ros::ok() ) {
			fprintf( stdout, "\n" );
			fprintf(stdout, " =>  %d. make planed paths publisher\n", ++phase);

			if( !node_config.topic_name_planned_path.value[0] ) {
				ros::shutdown();
				fprintf(stdout, "    ... error: invalid topic name\n");
			}
			else {
				fprintf(stdout, "    ... topic name is \"%s\"\n", node_config.topic_name_planned_path.value );
				pub_planned_path = nodehandle.advertise<msg_path_t>(node_config.topic_name_planned_path.value, 5 );

				fprintf(stdout, "    ... ok\n");
			}
		} // <--- initialize paths publisher

		// ---> initialize vehicle control command publisher
//		if( ros::ok() ) {
//			fprintf( stdout, "\n" );
//			fprintf(stdout, " =>  %d. make vehicle control command publisher\n", ++phase);
//
//			if( !node_config.topic_name_vehicle_ctrl.value[0] ) {
//				ros::shutdown();
//				fprintf(stdout, "    ... error: invalid topic name\n");
//			}
//			else {
//				fprintf(stdout, "    ... topic name is \"%s\"\n", node_config.topic_name_vehicle_ctrl.value );
//				pub_vehicle_ctrl = nodehandle.advertise<msg_vehicle_ctrl_t>(node_config.topic_name_vehicle_ctrl.value, 2 );
//
//				msg_vehicle_ctrl.header.stamp = time_start;
//				msg_vehicle_ctrl.header.seq = 0;
//				msg_vehicle_ctrl.header.frame_id = node_config.node_name.value;
//
//				fprintf(stdout, "    ... ok\n");
//			}
//		} // <--- initialize vehicle control command publisher

		{ // ---> set vehicle velocity
			Spur_set_vel( node_config.max_velocity.value );
			Spur_set_accel( 0.3 );
			Spur_set_angvel( node_config.max_angular_velocity.value );
			Spur_set_angaccel( M_PI / 2.0 );
		} // <--- set vehicle velocity

		// ---> make controller subscriber
		if( ros::ok() && node_config.topic_name_controller.value[0]) {
			fprintf(stderr, "\n");
			fprintf(stdout, " => %d. make controller subscriber for obstacle detection\n", ++phase);

			if( !node_config.topic_name_controller.value[0] ) {
				fprintf(stdout, "    ... error: invalid topic name\n");
				ros::shutdown();
			}
			else {
				fprintf(stdout, "    ... topic name is \"%s\"\n", node_config.topic_name_controller.value );

				// allocate buffer
				msgreader_controller.allocate(200);

				msg_controller.header.seq = 0;
				msg_controller.header.stamp.fromSec(0.0);

				msg_controller_prev.header.seq = 0;
				msg_controller_prev.header.stamp.fromSec(0.0);

				// subscribe
				subsc_controller = nodehandle.subscribe(node_config.topic_name_controller.value, 1000,
						&msgreader_controller_t::rosmsg_read,
						msgreader_controller.reader_pointer() );

				fprintf(stdout, "    ... ok\n");
			}
		} // <--- make controller subscriber


		// ---> make companion pose subscriber
		if( ros::ok() && node_config.topic_name_pose_companion.value[0]) {
			fprintf(stderr, "\n");
			fprintf(stdout, " => %d. make companion pose subscriber to accompany\n", ++phase);

			if( !node_config.topic_name_pose_companion.value[0] ) {
				fprintf(stdout, "    ... error: invalid topic name\n");
				ros::shutdown();
			}
			else {
				fprintf(stdout, "    ... topic name is \"%s\"\n", node_config.topic_name_pose_companion.value );

				// allocate buffer
				msgreader_pose_companion.allocate(100);

				msg_pose_companion.header.seq = 0;
				msg_pose_companion.header.stamp.fromSec(0.0);

				// subscribe
				subsc_pose_companion = nodehandle.subscribe(node_config.topic_name_pose_companion.value, 100,
						&msgreader_pose_companion_t::rosmsg_read,
						msgreader_pose_companion.reader_pointer() );

				fprintf(stdout, "    ... ok\n");
			}
		} // <--- make companion pose subscriber


		// ---> initialize target trajectory publisher
		if( ros::ok() ) {
			fprintf( stdout, "\n" );
			fprintf(stdout, " =>  %d. make the target trajectory publisher\n", ++phase);

			if( !node_config.topic_name_trajectory_target.value[0] ) {
				ros::shutdown();
				fprintf(stdout, "    ... error: invalid topic name\n");
			}
			else {
				fprintf(stdout, "    ... topic name is \"%s\"\n", node_config.topic_name_trajectory_target.value );
				pub_trajectory_target = nodehandle.advertise<msg_trajectory_t>(node_config.topic_name_trajectory_target.value, 2 );

				fprintf(stdout, "    ... ok\n");
			}
		} // <--- initialize target trajectory publisher


		// ---> initialize actual trajectory publisher
		if( ros::ok() ) {
			fprintf( stdout, "\n" );
			fprintf(stdout, " =>  %d. make actual trajectory publisher\n", ++phase);

			if( !node_config.topic_name_trajectory_actual.value[0] ) {
				ros::shutdown();
				fprintf(stdout, "    ... error: invalid topic name\n");
			}
			else {
				fprintf(stdout, "    ... topic name is \"%s\"\n", node_config.topic_name_trajectory_actual.value );
				pub_trajectory_actual = nodehandle.advertise<msg_trajectory_t>(node_config.topic_name_trajectory_actual.value, 2 );

				fprintf(stdout, "    ... ok\n");
			}
		} // <--- initialize actual trajectory publisher


		// ---> initialize vehicle status publisher
		if( ros::ok() ) {
			fprintf( stdout, "\n" );
			fprintf(stdout, " =>  %d. make vehiclle status publisher\n", ++phase);

			if( !node_config.topic_name_vehicle_status.value[0] ) {
				ros::shutdown();
				fprintf(stdout, "    ... error: invalid topic name\n");
			}
			else {
				fprintf(stdout, "    ... topic name is \"%s\"\n", node_config.topic_name_vehicle_status.value );
				pub_vehicle_stauts = nodehandle.advertise<msg_vehicle_status_t>(node_config.topic_name_vehicle_status.value, 5 );

				fprintf(stdout, "    ... ok\n");
			}
		} // <--- initialize vehicle status publisher

		// ---> make mode status subscriber
		if( ros::ok() && node_config.topic_name_mode_status.value[0] ){
			fprintf(stderr, "\n");
			fprintf(stdout, " => %d. make mode status subscriber\n", ++phase);

			if( !node_config.topic_name_mode_status.value[0] ){
				fprintf(stdout, "   ... error:invalid topic name\n");
				ros::shutdown();
			}
			else{
				fprintf(stdout, "   ... topic name is \"%s\"\n", node_config.topic_name_mode_status.value);

				// allocate buffer
				msgreader_mode_status.allocate(50);

				msg_mode_status.header.seq = 0;
				msg_mode_status.header.stamp.fromSec(0.0);

				// subscribe
				subsc_mode_status = nodehandle.subscribe(node_config.topic_name_mode_status.value, 100,
						&msgreader_mode_status_t::rosmsg_read,
						msgreader_mode_status.reader_pointer());

				fprintf(stdout, "   ... ok\n");
			}
		} // <--- make mode status subscriber

		// ---> make external controll subscriber
		if( ros::ok() && node_config.topic_name_external_controll_velocity.value[0] ){
			fprintf(stderr, "\n");
			fprintf(stdout, " => %d. make external_controll_velocity subscriber\n", ++phase);

			if( !node_config.topic_name_external_controll_velocity.value[0] ){
				fprintf(stdout, "   ... error:invalid topic name\n");
				ros::shutdown();
			}
			else{
				fprintf(stdout, "   ... topic name is \"%s\"\n", node_config.topic_name_external_controll_velocity.value);
				external_controlled_velocity = node_config.max_velocity.value;
				// subscribe
				subsc_external_controll_velocity = nodehandle.subscribe(node_config.topic_name_external_controll_velocity.value, 100,
					+[](const std_msgs::Float64::ConstPtr& msg) { 
						external_controlled_velocity = msg->data;
    					return;
					}
				);
				fprintf(stdout, "   ... ok\n");
			}
		} // <--- make mode status subscriber

		// ---> make service server
		if( ros::ok() ) {
			fprintf(stdout, "\n");
			fprintf(stdout, " => %d. make service server to reset particles\n", ++phase);

			if( !node_config.service_name_set_navigation_path.value[0] ) {
				fprintf(stdout, "    ... error: invalid service name\n");
				ros::shutdown();
			}
			else {
				fprintf(stdout, "    ... service name is \"%s\"\n", node_config.service_name_set_navigation_path.value );

				// service server
				srv_funcobj_set_navigation_path.clear();
				srvserver_set_navigation_path = nodehandle.advertiseService(
						node_config.service_name_set_navigation_path.value,
						&srv_funcobj_set_navigation_path_t::callback, &srv_funcobj_set_navigation_path);
				fprintf(stdout, "    ... ok\n");
			}
		} // <--- make service server

	} // <--- initialize



	if( ros::ok() ){ // ---> operate
		int end_path_id;
		typedef struct point_t {
			double x;
			double y;
		} point_t;
		typedef struct trajectory_evaluation_elements {
			double clearance;
			double depth;
			double dir;
			double dist_to_end;
		} trajectory_evaluation_elements_t;
		typedef struct obstacle_for_line_and_turn_t {
			double depth;
			double clearance_dist;
		} obstacle_for_line_and_turn_t;

		bool flagMovable = false;
		bool flagManual = false;
		bool flagNotControl = false;

		bool flagAutoMode = false;
		//const double timeWatchdogReference = 0.25;

		const int Flags_ErrorState_LostPoisiton = 0x00000001;
		const int Flags_ErrorState_LostPointcloud = 0x00000002;
		const int Flags_ErrorState_LostCompanion = 0x00000004;

		const double Period_ErrorState_LostPosition = 0.25;
		const double Period_ErrorState_LostPointcloud = 0.25;
		const double Period_ErrorState_LostCompanion = 1.0;

		ros::Rate loop_rate(1000);

		double var_debug_double = 0;
		// time
		double time_start;
		double time_current;
		// coordinate
		gnd::matrix::coord_tree coordinate_tree;
		int coordinate_id_global = -1;
		int coordinate_id_robot = -1;
		int coordinate_id_pathend = -1;
		int coordinate_id_trajectory_target = -1;
		int coordinate_id_trajectory_actual = -1;

		gnd::queue<point_t> pointcloud_on_pathend;

		// planning
		obstacle_for_line_and_turn_t		obstalce_for_motion;
		gnd::path::trajectory_t 			trajectory_target;
		trajectory_evaluation_elements_t	eval_trajectory_target_for_velocity_determination;
		gnd::path::trajectory_t				trajectory_actual;
		trajectory_evaluation_elements_t	eval_trajectory_actual_for_velocity_determination;
		double search_range_right, search_range_left;
		double schedule_planning;
		double schedule_reselect_trajecotry;
		double schedule_planning_trajecotry_actual;
		double schedule_publish_planning;
		double time_pose_update;
		double time_pose_companion_update;
		double time_pointcloud_update;
		double velocity = 0.0;
		double prev_velocity = 0.0;


		int flgs_errorstate = 0x00000000;

		gnd::queue<trajectory_evaluation_elements_t> eval_trajectories;


		// send commands
		double schedule_publish_planed_path;
		double period_publish_planed_path = 3.0;

		// terminal display
		int nline_display = 0;
		double schedule_display;

		{ // ---> initialize time variables
			time_start = ros::Time::now().toSec();
			time_current = time_start;
		} // <--- initialize time variables

		{ // ---> initialize message header
			for( int i = 0; i < ntopics_pointcloud; i++){
				topics_pointcloud[i].msg.header.seq = 0;
				topics_pointcloud[i].msg.header.stamp.fromSec(0.0);
			}
			msg_pose.header.seq = 0;
			msg_pose.header.stamp.fromSec(0.0);

			msg_velocity.header.seq = 0;
			msg_velocity.header.stamp.fromSec(0.0);
		} // <--- initialize message header

		{ // ---> initialize variables for planning
			trajectory_target.lines.clear();
			trajectory_target.lines.allocate(1);
			trajectory_actual.lines.clear();
			trajectory_actual.lines.allocate(1);
			schedule_reselect_trajecotry = time_start + node_config.period_reselect_trajectory.value;
			schedule_planning_trajecotry_actual = time_start;
			schedule_planning = time_start;
			time_pose_update = 0;
			time_pose_companion_update = 0;
			time_pointcloud_update = 0;
			flgs_errorstate = Flags_ErrorState_LostPoisiton | Flags_ErrorState_LostPointcloud;
			schedule_publish_planning = time_start;
		} // <--- initialize variables for planning

		{ // ---> initialize variables to send commands
			schedule_publish_planed_path = time_start;
		} // <--- initialize variables to send commands

		{ // ---> initialize coordinate
			gnd::matrix::fixed<4,4>  coord_tf_matrix;

			// define global coordinate
			gnd::matrix::set_unit(&coord_tf_matrix);
			coordinate_id_global = coordinate_tree.add("global", "root", &coord_tf_matrix);

			// initialize robot coordinate
			gnd::matrix::set_unit(&coord_tf_matrix);
			coordinate_id_robot = coordinate_tree.add("robot", "global", &coord_tf_matrix);

			// initialize path coordinate
			if( msg_planned_path.path.size() > 0 ) {
				gnd::matrix::coordinate_converter(&coord_tf_matrix,
					msg_planned_path.path[0].end.x, msg_planned_path.path[0].end.y, 0.0,
					cos(msg_planned_path.path[0].end.theta), sin(msg_planned_path.path[0].end.theta), 0.0,
					0.0, 0.0, 1.0);
			}
			else {
				gnd::matrix::set_unit(&coord_tf_matrix);
			}
			coordinate_tree.set_coordinate( coordinate_id_pathend, &coord_tf_matrix );
			coordinate_id_pathend = coordinate_tree.add("path", "global", &coord_tf_matrix);

			// initialize trajectory coordinate
			gnd::matrix::set_unit(&coord_tf_matrix);
			coordinate_id_trajectory_target = coordinate_tree.add("trajectory_target", "global", &coord_tf_matrix);

			// initialize trajectory coordinate
			gnd::matrix::set_unit(&coord_tf_matrix);
			coordinate_id_trajectory_actual = coordinate_tree.add("trajectory_actual", "global", &coord_tf_matrix);
		} // <--- initialize coordinate

		{ // ---> initialize variables for terminal display
			schedule_display = time_start;
			nline_display = 0;
		} // <--- initialize variables for terminal display

		// f陣形モードのサブスクライブ
 		ros::Subscriber sub_formation_mode = nodehandle.subscribe("formation_topic", 1, formationModeCallback);

		// ---> main loop
		fprintf(stdout, " => main loop start\n");
		while( ros::ok() ) {
			gnd::vector::fixed_column<2> robot_on_pathend;
			gnd::vector::fixed_column<2> companion_on_pathend;
			double theta_robot_on_pathend = 0.0;

			// blocking to avoid the waste of computing resource
			loop_rate.sleep();

			// spin
			ros::spinOnce();

			// save time
			time_current = ros::Time::now().toSec();


			// F陣形モードの判別
			if (formation_mode)	{

                double angle = fabs(formation_angle);
				Spur_set_vel(formation_linear_velocity);
				Spur_set_accel(0.3);
                Spur_set_angvel(angle);
				Spur_set_angaccel(1.0);
				Spur_set_pos_GL(0, 0, 0);
                Spur_line_GL(0, 0, formation_angle);
				ROS_INFO("Moving robot\n");
			}
			else{
			// ---> set planned path
			if( srv_funcobj_set_navigation_path.is_called() ) {
				// set path
				msg_planned_path.path.clear();
				srv_funcobj_set_navigation_path.get_path(&msg_planned_path);

				// clear
				srv_funcobj_set_navigation_path.clear();

				// publish new path
				pub_planned_path.publish(msg_planned_path);

				if( msg_planned_path.path.size() > 0 ) { // ---> update the current path coordinate
					gnd::matrix::fixed<4,4> coord_tf_matrix;

					gnd::matrix::coordinate_converter(&coord_tf_matrix,
							msg_planned_path.path[0].end.x, msg_planned_path.path[0].end.y, 0.0,
							cos(msg_planned_path.path[0].end.theta), sin(msg_planned_path.path[0].end.theta), 0.0,
							0.0, 0.0, 1.0);
					coordinate_tree.set_coordinate( coordinate_id_pathend, &coord_tf_matrix );
				} // <--- update the current path coordinate

				trajectory_target.lines.clear();
				trajectory_actual.lines.clear();

			} // ---> set planned path
			else if( node_config.test_mode.value ) {
				msg_planned_path.path.resize(1);
				msg_planned_path.start.x = -0.5;
				msg_planned_path.start.y = 0.0;
				msg_planned_path.start.theta = 0.0;

				msg_planned_path.path[0].end.x = 10.0;
				msg_planned_path.path[0].end.y = 0.0;
				msg_planned_path.path[0].end.theta = gnd_deg2rad(0.0);
				msg_planned_path.path[0].curvature = 0.0;

				msg_planned_path.path[0].left_width = 1.0;
				msg_planned_path.path[0].right_width = 1.0;
				msg_planned_path.path[0].start_extend = 1.0;
				msg_planned_path.path[0].end_extend = 1.0;

				{ // ---> update the current path coordinate
					gnd::matrix::fixed<4,4>  coord_tf_matrix;

					gnd::matrix::coordinate_converter(&coord_tf_matrix,
							msg_planned_path.path[0].end.x, msg_planned_path.path[0].end.y, 0.0,
							cos(msg_planned_path.path[0].end.theta), sin(msg_planned_path.path[0].end.theta), 0.0,
							0.0, 0.0, 1.0);
					coordinate_tree.set_coordinate( coordinate_id_pathend, &coord_tf_matrix );
				} // <--- update the current path coordinate

			}


			// ---> upper task
			if( node_config.topic_name_controller.value[0] && time_current > schedule_planning ) {
				// ---> get user controll
				if( msgreader_controller.copy_new(&msg_controller, &msg_controller.header.stamp) != 0 ) {
					if ( msg_controller_prev.header.stamp.toSec() + node_config.time_watchdog_reference.value < time_current ) {
						flagMovable = false;
					}
				} else {
					// user control
					if (msg_controller.buttons.size() == LengthButtons && msg_controller.buttons[IndexL1]) {
						flagMovable = true;
					} else {
						flagMovable = false;
					}
					if (msg_controller.buttons.size() == LengthButtons && (msg_controller.buttons[IndexR1] || msg_controller.buttons[IndexShare])) {
						flagManual = true;
					} else {
						flagManual = false;
					}

					if (msg_controller.buttons.size() == LengthButtons && msg_controller.buttons[IndexOptions]) {
						flagNotControl = true;
					} else {
						flagNotControl = false;
					}




					// msg_controller
					msg_controller_prev = msg_controller;
				} // <--- get user controll

			}

			if( node_config.topic_name_mode_status.value[0] && time_current > schedule_planning ) {
				// ---> get mode status
				if( msgreader_mode_status.copy_new(&msg_mode_status, &msg_mode_status.header.stamp) != 0 ) {
				}
				else{
					if( msg_mode_status.status == msg_mode_status.MODE_STATE_AUTO ){
						flagAutoMode = true;
					}
					else {
						flagAutoMode = false;
					}
				}
			} else if ( !node_config.topic_name_mode_status.value[0] ){
				flagAutoMode = true;
			}
			// ---> upper task

			

			// ---> planning
			if( time_current > schedule_planning ) {
				double pointcloud_stamp = time_current;
				double length_current_path;
				double cos_on_current_pathend, sin_on_current_pathend;

				{ // ---> obtain data
					// ---> get self-position
					if( msgreader_pose.copy_new(&msg_pose, &msg_pose.header.stamp) != 0 ) {
						// test mode
						if( node_config.test_mode.value)  {
							msg_pose.x = 0;
							msg_pose.y = 0;
							msg_pose.theta = gnd_deg2rad(0.0);
							time_pose_update = time_current;
						}
					}
					else {
						// update
						time_pose_update = time_current;
					} // <--- get self-position


					// ---> check lost position or not
					if( time_current - time_pose_update > Period_ErrorState_LostPosition ) {
						// error: lost self position
						flgs_errorstate |= Flags_ErrorState_LostPoisiton;
					}
					else {
						gnd::matrix::fixed<4,4>  coord_tf_matrix;

						flgs_errorstate &= ~Flags_ErrorState_LostPoisiton;

						// define current robot coordinate
						gnd::matrix::coordinate_converter(&coord_tf_matrix,
								msg_pose.x, msg_pose.y, 0.0,
								cos(msg_pose.theta), sin(msg_pose.theta), 0.0,
								0.0, 0.0, 1.0);
						coordinate_tree.set_coordinate( coordinate_id_robot, &coord_tf_matrix);
					} // <--- check lost position or not

					// ---> calculate the robot position on current path
					if( !(flgs_errorstate & Flags_ErrorState_LostPoisiton) && msg_planned_path.path.size() == 0) {
						// path end
						robot_on_pathend[0] = 0.0;
						robot_on_pathend[1] = 0.0;
						theta_robot_on_pathend = 0.0;
					}
					else if( !(flgs_errorstate & Flags_ErrorState_LostPoisiton) && (signed) msg_planned_path.path.size() > 0) {
						// calculate robot position on path
						cos_on_current_pathend = cos( msg_planned_path.path[0].end.theta );
						sin_on_current_pathend = sin( msg_planned_path.path[0].end.theta );

						robot_on_pathend[0] = cos_on_current_pathend * ( msg_pose.x - msg_planned_path.path[0].end.x )
										+ sin_on_current_pathend * ( msg_pose.y - msg_planned_path.path[0].end.y );
						robot_on_pathend[1] = - sin_on_current_pathend * ( msg_pose.x - msg_planned_path.path[0].end.x )
										+ cos_on_current_pathend * ( msg_pose.y - msg_planned_path.path[0].end.y );
					}
					else {
					} // <--- calculate the robot position on current path


					// ---> get companion position
					if( node_config.topic_name_pose_companion.value[0] ) {
						if( msgreader_pose_companion.copy_new(&msg_pose_companion, &msg_pose_companion.header.stamp) != 0 ) {

						}
						else {
							time_pose_companion_update = msg_pose_companion.header.stamp.toSec();
						}

						// check companion data
						if( time_current - time_pose_companion_update > Period_ErrorState_LostCompanion) {
							companion_on_pathend[0] = 0.0;
							companion_on_pathend[1] = 0.0;

							flgs_errorstate |= Flags_ErrorState_LostCompanion;
						}
						else {
							gnd::matrix::fixed<4,4>  coord_tf_matrix;
							gnd::vector::fixed_column<4> companion_on_robot;

							companion_on_robot[0] = msg_pose_companion.x;
							companion_on_robot[1] = msg_pose_companion.y;
							companion_on_robot[2] = 0.0;
							companion_on_robot[3] = 1.0;

							// coordinate transform
							coordinate_tree.get_convert_matrix(coordinate_id_robot, coordinate_id_pathend, &coord_tf_matrix);
							gnd::matrix::prod(&coord_tf_matrix, &companion_on_robot, &companion_on_pathend);

							flgs_errorstate &= ~Flags_ErrorState_LostCompanion;
						}
					}// <--- get companion position


					{ // ---> get point cloud data
						bool exist_new_data = true;
						// ---> checking that new data exist
						for( int i = 0; i < ntopics_pointcloud; i++){
							if( topics_pointcloud[i].msgreader.copy_new(&topics_pointcloud[i].msg, &topics_pointcloud[i].msg.header.stamp) != 0 ) {
								// no new data
								exist_new_data = false;
							}
							else {
								double stamp = topics_pointcloud[i].msg.header.stamp.toSec();
								// save the oldest time stamp
								pointcloud_stamp = pointcloud_stamp < stamp ? pointcloud_stamp : stamp;
							}
						} // <--- checking that new data exist

						// ---> get data associated with the time (some laser-scanner is not working case)
						if( !exist_new_data ) {
							// no new data
							if( node_config.test_mode.value ) {
								time_pointcloud_update = time_current;
							} // <--- test mode
						} // <--- get data associated with the time (some laser-scanner is not working case)
						// ---> get data associated with the time (some laser-scanners case)
						else if( ntopics_pointcloud > 1 ) {
							for( int i = 0; i < ntopics_pointcloud; i++){
								// get data of the time
								if( topics_pointcloud[i].msgreader.copy_at_time( &topics_pointcloud[i].msg, pointcloud_stamp) != 0 ) {
								}
								else {
								}
							}
							// update the latest point-cloud time
							time_pointcloud_update = time_current;
						} // <--- get data associated with the time (some laser-scanners case)
						// ---> get data associated with the time (only one laser-scanner case)
						else {
							// not need the association, only update the latest point-cloud time
							time_pointcloud_update = time_current;
						} // <--- get data associated with the time (only one laser-scanner case)
					} // <--- get point cloud data

					if( time_current - time_pointcloud_update > Period_ErrorState_LostPointcloud ) {
						// error: lost self position
						flgs_errorstate |= Flags_ErrorState_LostPointcloud;
					}
					else {
						flgs_errorstate &= ~Flags_ErrorState_LostPointcloud;
					}
				} // <--- obtain data


				// ---> path determination
				if( !flgs_errorstate ) {
					char flgs_transit_next = 0x00;
					const char flg_transit_next = 0x01;
					const char flg_transit_next_out_of_path = 0x02;
					const char flg_transit_next_end_of_path = 0x04;
					// transit the next path or not

					// ---> when there is no path
					if( (signed) msg_planned_path.path.size() == 0 ) {
						// do nothing
					} // <--- when there is no path
					// ---> when the current path is last one
					else if( (signed) msg_planned_path.path.size() == 1 ) {

						// when the robot arrive at the current path-end
						if( robot_on_pathend[0] > - node_config.pathend_margin.value ) {
							// transit to the next path
							flgs_transit_next |= flg_transit_next;
							flgs_transit_next |= flg_transit_next_end_of_path;
						}
					} // <--- when the current path is last one
					// ---> when some paths area
					else if( (signed) msg_planned_path.path.size() > 1 ) {
						// check the distance to the path-end extend boundary
						double length_next_path;
						double x_robot_on_next_pathend, y_robot_on_next_pathend;
						double cos_on_next_pathend, sin_on_next_pathend;

						// calculate robot position on path
						cos_on_next_pathend = cos( msg_planned_path.path[1].end.theta );
						sin_on_next_pathend = sin( msg_planned_path.path[1].end.theta );

						x_robot_on_next_pathend = cos_on_next_pathend * ( msg_pose.x - msg_planned_path.path[1].end.x )
										+ sin_on_next_pathend * ( msg_pose.y - msg_planned_path.path[1].end.y );
						y_robot_on_next_pathend = - sin_on_next_pathend * ( msg_pose.x - msg_planned_path.path[1].end.x )
										+ cos_on_next_pathend * ( msg_pose.y - msg_planned_path.path[1].end.y );
						length_next_path = cos_on_next_pathend * ( msg_planned_path.path[1].end.x - msg_planned_path.path[0].end.x )
										+ sin_on_next_pathend * ( msg_planned_path.path[1].end.y - msg_planned_path.path[0].end.y );

						// when the robot is in the next path area
						if( x_robot_on_next_pathend > - length_next_path - msg_planned_path.path[1].start_extend &&
							x_robot_on_next_pathend <   msg_planned_path.path[1].end_extend &&
							y_robot_on_next_pathend > - msg_planned_path.path[1].right_width &&
							y_robot_on_next_pathend <   msg_planned_path.path[1].left_width ) {
							// transit to the next path
							flgs_transit_next |= flg_transit_next;
						}
						// when the robot arrives at the current path-end
						else if( robot_on_pathend[0] > - node_config.pathend_margin.value ||
								 robot_on_pathend[0] > msg_planned_path.path[0].end_extend - (node_config.vehicle_length_back.value + node_config.clearance_required.value)) {
							// transit to the next path
							flgs_transit_next |= flg_transit_next;
							flgs_transit_next |= flg_transit_next_out_of_path;
						}

					} // <--- when some paths area


					// ---> operation that transit to the next path
					if( flgs_transit_next ) {

						// transit to the next path
						msg_planned_path.start = msg_planned_path.path[0].end;
						msg_planned_path.path.erase( msg_planned_path.path.begin() );
						trajectory_target.lines.clear();
						trajectory_actual.lines.clear();

						// update
						if ( (signed) msg_planned_path.path.size() > 0 ) {
							gnd::matrix::fixed<4,4>  coord_tf_matrix;

							{ // ---> update the current path coordinate
								gnd::matrix::coordinate_converter(&coord_tf_matrix,
										msg_planned_path.path[0].end.x, msg_planned_path.path[0].end.y, 0.0,
										cos(msg_planned_path.path[0].end.theta), sin(msg_planned_path.path[0].end.theta), 0.0,
										0.0, 0.0, 1.0);
								coordinate_tree.set_coordinate( coordinate_id_pathend, &coord_tf_matrix );
							} // <--- update the current path coordinate

							{ // ---> update target trajectory
								double diff_theta;
								double y_robot_on_next_pathend;

								y_robot_on_next_pathend = - sin( msg_planned_path.path[0].end.theta ) * ( msg_pose.x - msg_planned_path.path[0].end.x )
												+ cos( msg_planned_path.path[0].end.theta ) * ( msg_pose.y - msg_planned_path.path[0].end.y );
								diff_theta = gnd_rad_normalize(msg_pose.theta - msg_planned_path.path[0].end.theta);


								// trajectory
								trajectory_target.lines.resize(1);
								if( fabs(diff_theta) < gnd_deg2rad(10.0) ) {
									double shift_path = y_robot_on_next_pathend;
									// when robot is out of next path
									if( shift_path > msg_planned_path.path[0].left_width - ( node_config.vehicle_width_left.value + node_config.clearance_required.value)  )  {
										shift_path = msg_planned_path.path[0].left_width - ( node_config.vehicle_width_left.value + node_config.clearance_required.value);
									}
									else if( shift_path < - msg_planned_path.path[0].right_width + ( node_config.vehicle_width_left.value + node_config.clearance_required.value) ){
										shift_path = - msg_planned_path.path[0].right_width + ( node_config.vehicle_width_left.value + node_config.clearance_required.value);
									}

									trajectory_target.lines[0].end.x = msg_planned_path.path[0].end.x + sin(msg_planned_path.path[0].end.theta) * shift_path;
									trajectory_target.lines[0].end.y = msg_planned_path.path[0].end.y + cos(msg_planned_path.path[0].end.theta) * shift_path;
									trajectory_target.lines[0].end.theta = msg_planned_path.path[0].end.theta;
								}
								else if( (y_robot_on_next_pathend > 0 && diff_theta < 0) ||
										(y_robot_on_next_pathend < 0 && diff_theta > 0) ) {
									trajectory_target.lines[0].end.x = msg_planned_path.path[0].end.x;
									trajectory_target.lines[0].end.y = msg_planned_path.path[0].end.y;
									trajectory_target.lines[0].end.theta = msg_planned_path.path[0].end.theta;
								}
								else {
									double shift_path = y_robot_on_next_pathend;
									// when robot is out of next path
									if( shift_path > msg_planned_path.path[0].left_width - ( node_config.vehicle_width_left.value + node_config.clearance_required.value)  )  {
										shift_path = msg_planned_path.path[0].left_width - ( node_config.vehicle_width_left.value + node_config.clearance_required.value);
									}
									else if( shift_path < - msg_planned_path.path[0].right_width + ( node_config.vehicle_width_left.value + node_config.clearance_required.value) ){
										shift_path = - msg_planned_path.path[0].right_width + ( node_config.vehicle_width_left.value + node_config.clearance_required.value);
									}

									trajectory_target.lines[0].end.x = msg_planned_path.path[0].end.x + sin(msg_planned_path.path[0].end.theta) * shift_path;
									trajectory_target.lines[0].end.y = msg_planned_path.path[0].end.y + cos(msg_planned_path.path[0].end.theta) * shift_path;
									trajectory_target.lines[0].end.theta = msg_planned_path.path[0].end.theta;
								}


								{ // ---> update the current path coordinate
									gnd::matrix::coordinate_converter(&coord_tf_matrix,
											trajectory_target.lines[0].end.x, trajectory_target.lines[0].end.y, 0.0,
											cos(trajectory_target.lines[0].end.theta), sin(trajectory_target.lines[0].end.theta), 0.0,
											0.0, 0.0, 1.0);
									coordinate_tree.set_coordinate( coordinate_id_trajectory_target, &coord_tf_matrix );
								} // <--- update the current path coordinate


							} // <--- update target trajectory


						}
						else {
							// do nothing
						}

					} // <--- operation that transit to the next path
				} // <--- path determination

			// 軌跡決定
			// ロボットの現在の位置と目標軌道（target trajectory）、および実際の軌道（actual trajectory）を基に、次に進むべき軌道を決定
			// 軌道の選択は、周囲の障害物の位置やその他の要素に基づく
				// ---> trajectory determination
				if( msg_planned_path.path.size() <= 0 ) {
					trajectory_actual.lines.clear();
					trajectory_target.lines.clear();
				}
				else if( !flgs_errorstate ) {
					bool flg_reselect_trajectory = false;
					double shift_from_pathline_to_trajectory;

					// if robot have no target trajectories
					// ---> set trajectory
					if( trajectory_target.lines.size() == 0 ) {
						gnd::matrix::fixed<4,4>  coord_tf_matrix;
						// trajectory
						trajectory_target.lines.resize(1);
						trajectory_target.lines[0].end.x = msg_planned_path.path[0].end.x;
						trajectory_target.lines[0].end.y = msg_planned_path.path[0].end.y;
						trajectory_target.lines[0].end.theta = msg_planned_path.path[0].end.theta;

						gnd::matrix::coordinate_converter(&coord_tf_matrix,
								trajectory_target.lines[0].end.x, trajectory_target.lines[0].end.y, 0.0,
								cos(trajectory_target.lines[0].end.theta), sin(trajectory_target.lines[0].end.theta), 0.0,
								0.0, 0.0, 1.0);
						coordinate_tree.set_coordinate( coordinate_id_trajectory_target, &coord_tf_matrix );
					} // <--- set trajectory


					{ // ---> coordinate transformation of point-cloud (from robot to path-end) and check obstacles on the current target trajectory
						gnd::matrix::fixed<4,4> 	 coord_tf_matrix;
						gnd::vector::fixed_column<4> point_on_robot;
						gnd::vector::fixed_column<4> point_on_pathend;
						point_t point;
						double search_range_right_from_robot, search_range_left_from_robot;

						length_current_path = - ( cos(msg_planned_path.path[0].end.theta) * ( msg_planned_path.start.x - msg_planned_path.path[0].end.x )
								+ sin(msg_planned_path.path[0].end.theta) * ( msg_planned_path.start.y - msg_planned_path.path[0].end.y ) );

						// coordinate convert matrix
						coordinate_tree.get_convert_matrix( coordinate_id_robot, coordinate_id_pathend, &coord_tf_matrix );
						shift_from_pathline_to_trajectory = -sin_on_current_pathend * ( trajectory_target.lines[0].end.x - msg_planned_path.path[0].end.x )
										+ cos_on_current_pathend * ( trajectory_target.lines[0].end.y - msg_planned_path.path[0].end.y );

						// ---> 1. extract the point-cloud data within the current path area
						//      2. coordinate transform from robot to path-end
						//      3. check obstacles on the current trajectory
						//      4. check obstacles in front of robot
						//      5. check trajectories to be able to connect
						// (for reduction of computing cost, it operates above 5 tasks simultaneously in a scanning loop)
						eval_trajectory_target_for_velocity_determination.depth = node_config.depth_for_trajectory_selection.value;
						eval_trajectory_target_for_velocity_determination.clearance = node_config.clearance_margin.value;
						eval_trajectory_target_for_velocity_determination.dir = 0.0;
						eval_trajectory_target_for_velocity_determination.dist_to_end = (- robot_on_pathend[0]);
						obstalce_for_motion.depth = node_config.depth_for_trajectory_selection.value;
						obstalce_for_motion.clearance_dist = node_config.clearance_margin.value;
						search_range_right_from_robot = -DBL_MAX / 2.0;
						search_range_left_from_robot = DBL_MAX / 2.0;
						pointcloud_on_pathend.clear();
						// ---> scanning loop (point cloud topics)
						for( int j = 0; j < ntopics_pointcloud; j++ ){
							// ---> scanning loop (points)
							for( int i = 0; i < (signed)topics_pointcloud[j].msg.points.size(); i++ ) {
								double depth, clearance;
								// ignore the points showing the robot itself
								if( topics_pointcloud[j].msg.points[i].x <   node_config.vehicle_length_front.value &&
										topics_pointcloud[j].msg.points[i].x > - node_config.vehicle_length_back.value &&
										topics_pointcloud[j].msg.points[i].y <   node_config.vehicle_width_left.value  &&
										topics_pointcloud[j].msg.points[i].y > - node_config.vehicle_width_right.value ) {
									continue;
								}

								{ // ---> check obstacles in front of robot
									// obstacles in front of the robot
									if( topics_pointcloud[j].msg.points[i].x >   node_config.vehicle_length_front.value &&
											topics_pointcloud[j].msg.points[i].y <   node_config.vehicle_width_left.value  &&
											topics_pointcloud[j].msg.points[i].y > - node_config.vehicle_width_right.value ) {

										obstalce_for_motion.depth = obstalce_for_motion.depth < (topics_pointcloud[j].msg.points[i].x - node_config.vehicle_length_front.value) ?
												obstalce_for_motion.depth : (topics_pointcloud[j].msg.points[i].x - node_config.vehicle_length_front.value);

									}

									// clearance of the straight trajectory in front of the robot
									if( topics_pointcloud[j].msg.points[i].x <   node_config.vehicle_length_front.value &&
											topics_pointcloud[j].msg.points[i].x > - node_config.vehicle_length_back.value ) {

										double clearance = topics_pointcloud[j].msg.points[i].y < 0 ? ( -topics_pointcloud[j].msg.points[i].y) - node_config.vehicle_width_right.value : topics_pointcloud[j].msg.points[i].y - node_config.vehicle_width_left.value;
										obstalce_for_motion.clearance_dist = clearance < obstalce_for_motion.clearance_dist ?
												clearance : obstalce_for_motion.clearance_dist;
									}
								} // <--- check obstacles in front of robot

								{ // ---> coordinate transform
									point_on_robot[0] = topics_pointcloud[j].msg.points[i].x;
									point_on_robot[1] = topics_pointcloud[j].msg.points[i].y;
									point_on_robot[2] = topics_pointcloud[j].msg.points[i].z;
									point_on_robot[3] = 1;

									gnd::matrix::prod(&coord_tf_matrix, &point_on_robot, &point_on_pathend);
								} // <--- coordinate transform

								{ // ---> extract the point-cloud data within the current path area

									if( topics_pointcloud[j].msg.points[i].x < node_config.vehicle_length_front.value + 5.0 * node_config.max_velocity.value &&
											topics_pointcloud[j].msg.points[i].x > -(node_config.vehicle_length_back.value + 5.0 * node_config.max_velocity.value) &&
											topics_pointcloud[j].msg.points[i].y < node_config.vehicle_width_left.value + 5.0 * node_config.max_velocity.value &&
											topics_pointcloud[j].msg.points[i].y > -(node_config.vehicle_width_left.value + 5.0 * node_config.max_velocity.value) ) {

										// with the exception, remain the points within the range that the robot is able to reach in 5 second
										// to do nothing
									}
									else {
										// remove data out of the current path range
										if( point_on_pathend[0] >  msg_planned_path.path[0].end_extend )							continue;
										if( point_on_pathend[0] < -msg_planned_path.path[0].start_extend - length_current_path )	continue;
										if( point_on_pathend[1] >  msg_planned_path.path[0].left_width )							continue;
										if( point_on_pathend[1] < -msg_planned_path.path[0].right_width )							continue;
									}

									point.x = point_on_pathend[0];
									point.y = point_on_pathend[1];

									// push back
									pointcloud_on_pathend.push_back( &point );
								} // <--- extract the point-cloud data within the current path area


								{ // ---> check obstacles on the current trajectory
									// calculate depth and clearance
									depth = point_on_pathend[0] - (robot_on_pathend[0] + node_config.vehicle_length_front.value);
									if( point_on_pathend[1] > shift_from_pathline_to_trajectory + (node_config.vehicle_width_left.value + node_config.clearance_required.value) ) {
										clearance = point_on_pathend[1] - (shift_from_pathline_to_trajectory + (node_config.vehicle_width_left.value + node_config.clearance_required.value));
									}
									else if(point_on_pathend[1] < shift_from_pathline_to_trajectory - (node_config.vehicle_width_right.value + node_config.clearance_required.value) ) {
										clearance = -point_on_pathend[1] + (shift_from_pathline_to_trajectory - (node_config.vehicle_width_right.value + node_config.clearance_required.value));
									}
									else {
										clearance = 0;
									}

									// calculate nearest depth and clearance
									if( point_on_pathend[0] < msg_planned_path.path[0].end_extend &&
											point_on_pathend[0] > robot_on_pathend[0] - node_config.vehicle_length_back.value) {

										// update nearest depth
										if( depth > 0 && depth < eval_trajectory_target_for_velocity_determination.dist_to_end - node_config.vehicle_length_front.value &&
												point_on_pathend[1] < shift_from_pathline_to_trajectory + (node_config.vehicle_width_left.value + node_config.clearance_required.value) &&
												point_on_pathend[1] > shift_from_pathline_to_trajectory - (node_config.vehicle_width_right.value + node_config.clearance_required.value) ) {

											eval_trajectory_target_for_velocity_determination.depth = depth < eval_trajectory_target_for_velocity_determination.depth ? depth : eval_trajectory_target_for_velocity_determination.depth;
										}
										// update clearance
										else if( depth > 0 &&  depth < eval_trajectory_target_for_velocity_determination.dist_to_end - node_config.vehicle_length_front.value &&
												depth < node_config.depth_for_slow_down.value ) {
											eval_trajectory_target_for_velocity_determination.clearance = clearance < eval_trajectory_target_for_velocity_determination.clearance ? clearance : eval_trajectory_target_for_velocity_determination.clearance;
										}
									}
								} // ---> check obstacles on the current trajectory


								// ---> check trajectories to be able to connect
								if( point_on_pathend[0] < robot_on_pathend[0] + node_config.vehicle_length_front.value &&
										point_on_pathend[0] > robot_on_pathend[0] - node_config.vehicle_length_back.value  ) {
									double r = point_on_pathend[1] - robot_on_pathend[1];

									if( r > 0 && search_range_left_from_robot < r )			search_range_left_from_robot = r;
									else if( r < 0 && search_range_right_from_robot > r )	search_range_right_from_robot = r;
								} // <--- check trajectories to be able to connect

							} // <--- scanning loop (points)
						} // <--- scanning loop (point cloud topics)

						// search range
						search_range_left = search_range_left_from_robot + robot_on_pathend[1];
						search_range_left = search_range_left < msg_planned_path.path[0].left_width ? search_range_left : msg_planned_path.path[0].left_width;
						search_range_right = search_range_right_from_robot + robot_on_pathend[1];
						search_range_right = search_range_right > -msg_planned_path.path[0].right_width ? search_range_right : -msg_planned_path.path[0].right_width;
						// add vehicle width and margin
						search_range_left -= (node_config.vehicle_width_left.value + node_config.clearance_required.value);
						search_range_right += (node_config.vehicle_width_right.value + node_config.clearance_required.value);

						// depth
						eval_trajectory_target_for_velocity_determination.depth = eval_trajectory_target_for_velocity_determination.depth < node_config.depth_for_stop.value ? 0 : eval_trajectory_target_for_velocity_determination.depth;
					}
					// <--- 1. extract the laser-scanner data within the current path area
					//      2. coordinate transform from robot to path-end
					//      3. check obstacles on the current trajectory
					//      4. check obstacles in front of robot
					//      5. check trajectories to be able to connect


					{ // ---> determinate to change the target trajectory or not
						// there is no data representing obstacle
						if( eval_trajectory_target_for_velocity_determination.depth > node_config.depth_for_slow_down.value ) {
							// reset trajectory reselect schedule
							schedule_reselect_trajecotry = time_current + node_config.period_reselect_trajectory.value;
						}

						// for some time, there are some data representing obstacle
						if( time_current > schedule_reselect_trajecotry ) {
							// trajectory reselect
							flg_reselect_trajectory = true;
						}
						// exception: no target trajecotry
						else if( msg_planned_path.path.size() > 0 && trajectory_target.lines.size() <= 0 ) {
							// trajectory reselect
							flg_reselect_trajectory = true;
						}
						else if( node_config.topic_name_controller.value[0] && flagManual ) {
							flg_reselect_trajectory = true;
						}
						else if( node_config.topic_name_controller.value[0] && flagNotControl ) {
							flg_reselect_trajectory = true;
						}

					} // <--- determinate to change the target trajectory or not

					// if change the target trajectory
					// ---> reselect the target trajectory
					if( flg_reselect_trajectory ) {
						int i_max_eval = -1;
						double max_evaluation = 0;
						double n = ( node_config.vehicle_width_right.value + node_config.vehicle_width_left.value + 2 * node_config.clearance_required.value ) / node_config.discretized_interval_of_target_trajectory.value;

						{ // ---> determine the candidate trajectories
							int n_trajectories;

							eval_trajectories.clear();

							// quantization : transform width into some line trajectories
							search_range_left = search_range_left < 0 ? - ceil( -search_range_left / node_config.discretized_interval_of_target_trajectory.value ) * node_config.discretized_interval_of_target_trajectory.value
									: ceil( search_range_left / node_config.discretized_interval_of_target_trajectory.value ) * node_config.discretized_interval_of_target_trajectory.value ;
							search_range_right = search_range_right > 0 ? ceil( search_range_right / node_config.discretized_interval_of_target_trajectory.value ) * node_config.discretized_interval_of_target_trajectory.value
									: - ceil( -search_range_right / node_config.discretized_interval_of_target_trajectory.value ) * node_config.discretized_interval_of_target_trajectory.value ;
							n_trajectories = ceil( ( (search_range_left - search_range_right) / node_config.discretized_interval_of_target_trajectory.value ) + 0.4999 );

							if( n_trajectories > 0 ) {
								eval_trajectories.resize(n_trajectories);
								for( int i = 0; i < (signed)eval_trajectories.size(); i++ ) {
									// initialize evaluation elements
									eval_trajectories[i].depth = node_config.depth_for_trajectory_selection.value;
									eval_trajectories[i].clearance = node_config.clearance_margin.value;
									eval_trajectories[i].dir = 0.0;
									eval_trajectories[i].dist_to_end = (- robot_on_pathend[0]);
								}
							}
						} // <--- determine the candidate trajectories


						// ---> calculation of the trajectory evaluation elements
						if( eval_trajectories.size() > 0 ) {
							// ---> depth
							for( int i = 0; i < (signed)pointcloud_on_pathend.size(); i++ ) {
								int j_begin, j_end;
								double depth;

								if( pointcloud_on_pathend[i].x < robot_on_pathend[0] + node_config.vehicle_length_front.value )											continue;
								if( pointcloud_on_pathend[i].y < search_range_right - node_config.vehicle_width_right.value - node_config.clearance_required.value )	continue;
								if( pointcloud_on_pathend[i].y > search_range_left + node_config.vehicle_width_left.value + node_config.clearance_required.value )		continue;

								j_begin = ((pointcloud_on_pathend[i].y - node_config.vehicle_width_right.value - node_config.clearance_required.value) - (search_range_right)) / node_config.discretized_interval_of_target_trajectory.value;
								j_begin = j_begin >= 0 ? j_begin : 0;
								j_end = ((pointcloud_on_pathend[i].y + node_config.vehicle_width_left.value + node_config.clearance_required.value) - (search_range_right)) / node_config.discretized_interval_of_target_trajectory.value;
								depth = pointcloud_on_pathend[i].x - (robot_on_pathend[0] + node_config.vehicle_length_front.value + node_config.clearance_required.value) - node_config.depth_for_slow_down.value;

								// evaluate depth
								for( int j = j_begin; j < j_end && j < (signed)eval_trajectories.size(); j++ ){
									if( depth < eval_trajectories[j].dist_to_end - (node_config.vehicle_length_front.value + node_config.clearance_required.value) &&
											eval_trajectories[j].depth > depth ) {
										if( depth < node_config.depth_for_stop.value ) {
											eval_trajectories[j].depth = 0;
											eval_trajectories[j].clearance = 0;
										}
										else {
											eval_trajectories[j].depth = depth;
										}
									}
								}
							} // <--- depth

							// ---> clearance
							for( int i = 0; i < (signed)pointcloud_on_pathend.size(); i++ ) {
								int j_begin, j_end;
								double depth;

								if( pointcloud_on_pathend[i].x < robot_on_pathend[0] + node_config.vehicle_length_front.value + node_config.clearance_required.value )										continue;
								if( pointcloud_on_pathend[i].y < search_range_right - node_config.vehicle_width_right.value - node_config.clearance_required.value - node_config.clearance_margin.value)	continue;
								if( pointcloud_on_pathend[i].y > search_range_left + node_config.vehicle_width_left.value + node_config.clearance_required.value - node_config.clearance_margin.value )		continue;

								j_begin = ((pointcloud_on_pathend[i].y - node_config.vehicle_width_right.value - node_config.clearance_required.value - node_config.clearance_margin.value) - (search_range_right)) / node_config.discretized_interval_of_target_trajectory.value;
								j_begin = j_begin >= 0 ? j_begin : 0;
								j_end = ((pointcloud_on_pathend[i].y + node_config.vehicle_width_left.value + node_config.clearance_required.value + node_config.clearance_margin.value) - (search_range_right)) / node_config.discretized_interval_of_target_trajectory.value;
								depth = pointcloud_on_pathend[i].x - (robot_on_pathend[0] + node_config.vehicle_length_front.value + node_config.clearance_required.value);

								// evaluate clearance
								for( int j = j_begin; j < j_end && j < (signed)eval_trajectories.size(); j++ ){
									if( eval_trajectories[j].depth > 0 && depth < eval_trajectories[j].depth &&
											depth < (eval_trajectories[j].dist_to_end + node_config.vehicle_length_front.value) ) {

										double y_trajectory = search_range_right + j * node_config.discretized_interval_of_target_trajectory.value;
										double clearance_left = pointcloud_on_pathend[i].y - (y_trajectory  - node_config.vehicle_width_right.value - node_config.clearance_required.value);
										double clearance_right = -(pointcloud_on_pathend[i].y - (y_trajectory  - node_config.vehicle_width_right.value - node_config.clearance_required.value));
										double clearance = clearance_left > clearance_right ? clearance_left : clearance_right;

										clearance = clearance < node_config.clearance_margin.value ? clearance : node_config.clearance_margin.value;
										eval_trajectories[j].clearance = eval_trajectories[j].clearance < clearance ? eval_trajectories[j].clearance : clearance;
									}
								}
							} // <--- clearance
						} // <--- calculation of trajectory evaluation elements

						// ---> select trajectory
						if( eval_trajectories.size() > 0 ) {
							double eval_depth, eval_clearance, eval_dist, eval;

							i_max_eval = -1;
							max_evaluation = 0;

							for( int i = 0; i < (signed)eval_trajectories.size(); i++ ) {
								if( eval_trajectories[i].depth <= 0 ){
									eval = 0;
								}
								else {
									eval_depth = eval_trajectories[i].depth / (node_config.depth_for_trajectory_selection.value - node_config.depth_for_slow_down.value);
									eval_clearance = eval_trajectories[i].clearance / node_config.clearance_margin.value;
									eval_dist = exp( - gnd_square( 1.0 / 1.5 *  fabs(search_range_right + node_config.discretized_interval_of_target_trajectory.value * i - robot_on_pathend[1]) ) );
									eval = node_config.target_trajectory_weight_depth.value * eval_depth
										+ node_config.target_trajecotry_weight_clearance.value * eval_clearance
										+ node_config.target_trajecotry_weight_distance.value * eval_dist;
								}

								if( max_evaluation < eval && eval > 0  ) {
									max_evaluation = eval;
									i_max_eval = i;
								}
							}
						} // <--- select trajectory

						// ---> set new target trajectory
						if( i_max_eval >= 0 ) {
							double d = search_range_right + node_config.discretized_interval_of_target_trajectory.value * i_max_eval;
							gnd::matrix::fixed<4,4>  coord_tf_matrix;

							trajectory_target.lines.resize(1);
							trajectory_target.lines[0].end.theta = msg_planned_path.path[0].end.theta;
							trajectory_target.lines[0].end.x = msg_planned_path.path[0].end.x - sin(trajectory_target.lines[0].end.theta) * d;
							trajectory_target.lines[0].end.y = msg_planned_path.path[0].end.y + cos(trajectory_target.lines[0].end.theta) * d;
							schedule_reselect_trajecotry = time_current + node_config.period_reselect_trajectory.value;

							eval_trajectory_target_for_velocity_determination.depth = eval_trajectories[i_max_eval].depth;
							eval_trajectory_target_for_velocity_determination.clearance = eval_trajectories[i_max_eval].clearance;
							eval_trajectory_target_for_velocity_determination.dir = eval_trajectories[i_max_eval].dir;

							// coordinate
							gnd::matrix::coordinate_converter(&coord_tf_matrix,
									trajectory_target.lines[0].end.x, trajectory_target.lines[0].end.y, 0.0,
									cos(trajectory_target.lines[0].end.theta), sin(trajectory_target.lines[0].end.theta), 0.0,
									0.0, 0.0, 1.0);
							coordinate_tree.set_coordinate( coordinate_id_trajectory_target, &coord_tf_matrix );

							// reset actual trajecotry
							trajectory_actual.lines.clear();
						} // ---> set new target trajectory

					} // <--- reselect the target trajectory

					// ---> determine the actual trajectory
					if( trajectory_target.lines.size() <= 0 ) {
						// no target
						trajectory_actual.lines.clear();
					}
					else  {
						gnd::matrix::fixed<4,4>  		coord_tf_matrix_global2target;
						gnd::matrix::fixed<4,4>  		coord_tf_matrix_target2pathend;
						gnd::vector::fixed_column<4>	pose_on_global;
						gnd::vector::fixed_column<4>	pose_on_trajectory_target;
						gnd::vector::fixed_column<4>	pose_on_pathend;

						coordinate_tree.get_convert_matrix(coordinate_id_global, coordinate_id_trajectory_target, &coord_tf_matrix_global2target);
						coordinate_tree.get_convert_matrix(coordinate_id_trajectory_target, coordinate_id_pathend, &coord_tf_matrix_target2pathend);
						{ // ---> calculate the distance to target trajectory
							pose_on_global[0] = msg_pose.x;
							pose_on_global[1] = msg_pose.y;
							pose_on_global[2] = msg_pose.theta;
							pose_on_global[3] = 1;

							gnd::matrix::prod(&coord_tf_matrix_global2target, &pose_on_global, &pose_on_trajectory_target);
							gnd::matrix::prod(&coord_tf_matrix_target2pathend, &pose_on_trajectory_target, &pose_on_pathend);
						} // <--- calculate the distance to target trajectory

						// ---> calculate the actual trajectory
						trajectory_actual.lines.clear();
						// ---> if robot is not near the target trajectory, actual trajectory is planned an intermediate trajectory
						if( fabs(pose_on_trajectory_target[1]) > node_config.distance_threshold_intermediate.value ) {
							gnd::queue<gnd::path::trajectory_unit_t> trajectories_searched_on_pathend;
							gnd::path::trajectory_unit_t ws;
							int size;
							double search_dir_size;
							double search_range_start;
							double search_range_end;
							int i_max_eval;
							double max_eval;
							double max_dist_to_end = 0;

							// calculate the search range
							search_dir_size = pose_on_trajectory_target[1] > 0 ?
										node_config.mediate_trajectory_search_angular_size.value :
										-node_config.mediate_trajectory_search_angular_size.value;

							search_range_start = pose_on_trajectory_target[1] > 0 ? -M_PI / 2.0 : M_PI / 2.0;
							search_range_end = atan2( -pose_on_trajectory_target[1], node_config.mediate_trajectory_search_range.value );
							if( search_range_end >  node_config.mediate_trajectory_search_angular_range.value ) search_range_end =   node_config.mediate_trajectory_search_angular_range.value;
							if( search_range_end < -node_config.mediate_trajectory_search_angular_range.value ) search_range_end = - node_config.mediate_trajectory_search_angular_range.value;

							size = (search_range_end - search_range_start) / search_dir_size + 1;
							search_range_end = search_range_start + (size - 1) * search_dir_size;
							if(search_dir_size < 0){
								double ws;
								search_dir_size = -search_dir_size;
								ws = search_range_end;
								search_range_end = search_range_start;
								search_range_start = ws;
							}

							// ---> create the candidate trajectories
							trajectories_searched_on_pathend.resize(size);
							eval_trajectories.clear();
							eval_trajectories.resize(size);
							for( int i = 0; i < (signed)trajectories_searched_on_pathend.size(); i++ ) {
								double dir_on_target;
								gnd::vector::fixed_column<4> trajectory_searched_on_target;
								gnd::vector::fixed_column<4> trajectory_searched_on_pathend;

								dir_on_target = gnd_rad_normalize( search_range_start + i * search_dir_size );
								if( fabs(dir_on_target - M_PI / 2.0) > M_PI * 1.0 / 180.0 ) {
									trajectory_searched_on_target[0] = pose_on_trajectory_target[0] - pose_on_trajectory_target[1] / tan(dir_on_target);
								}
								else {
									trajectory_searched_on_target[0] = pose_on_trajectory_target[0];
								}
								trajectory_searched_on_target[1] = 0;
								trajectory_searched_on_target[2] = 0;
								trajectory_searched_on_target[3] = 1;

								gnd::matrix::prod(&coord_tf_matrix_target2pathend, &trajectory_searched_on_target, &trajectory_searched_on_pathend);

								// calculate on global coordinate
								trajectories_searched_on_pathend[i].curvature = 0;
								trajectories_searched_on_pathend[i].end.theta = gnd_rad_normalize( dir_on_target + (msg_planned_path.path[0].end.theta - trajectory_target.lines[0].end.theta) );
								trajectories_searched_on_pathend[i].end.x = trajectory_searched_on_pathend[0];
								trajectories_searched_on_pathend[i].end.y = trajectory_searched_on_pathend[1];

								// initialize
								eval_trajectories[i].depth = node_config.depth_for_trajectory_selection.value;
								eval_trajectories[i].dist_to_end = sqrt( gnd_square(trajectory_searched_on_pathend[0] - pose_on_pathend[0]) + gnd_square(trajectory_searched_on_pathend[1] - pose_on_pathend[1]) );
								eval_trajectories[i].dir = trajectories_searched_on_pathend[i].end.theta;
								eval_trajectories[i].clearance = node_config.clearance_margin.value;

								max_dist_to_end = max_dist_to_end > eval_trajectories[i].dist_to_end ? max_dist_to_end : eval_trajectories[i].dist_to_end;
							} // <--- create the candidate trajectories


							// ---> evaluate the clearance
							for( int i = 0; i < (signed)pointcloud_on_pathend.size(); i++ ) {
								double r_point, theta_point;
								double search_range_right;
								double search_range_left;
								int j_begin, j_end;

								r_point = sqrt( gnd_square(pointcloud_on_pathend[i].x - pose_on_pathend[0]) + gnd_square(pointcloud_on_pathend[i].y - pose_on_pathend[1]) );
								if( r_point > max_dist_to_end + node_config.vehicle_length_front.value + node_config.clearance_required.value ) continue;

								theta_point = atan2(pointcloud_on_pathend[i].y - pose_on_pathend[1], pointcloud_on_pathend[i].x - pose_on_pathend[0]);
								// search range right
								if( r_point == 0 || r_point <= node_config.vehicle_width_right.value + node_config.clearance_required.value + node_config.clearance_margin.value ) {
									search_range_right = theta_point - M_PI / 2.0;
								}
								else {
									search_range_right = theta_point - asin( (node_config.vehicle_width_right.value + node_config.clearance_required.value + node_config.clearance_margin.value) / r_point );
								}
								if( search_range_start < search_range_right ) {
									j_begin = (search_range_right - search_range_start) / search_dir_size;
								}
								else {
									j_begin = 0;
								}

								// search range left
								if( r_point == 0 || r_point <= node_config.vehicle_width_left.value + node_config.clearance_required.value + node_config.clearance_margin.value ) {
									search_range_left = theta_point + M_PI / 2.0;
								}
								else {
									search_range_left = theta_point + asin( (node_config.vehicle_width_left.value + node_config.clearance_required.value + node_config.clearance_margin.value) / r_point );
								}
								if( search_range_end > search_range_left ) {
									j_end = (search_range_left - search_range_start) / search_dir_size + 1;
								}
								else {
									j_end = trajectories_searched_on_pathend.size();
								}

								for( int j = j_begin; j < j_end; j++ ) {
									double dir;
									double clearance;

									if( r_point > eval_trajectories[j].dist_to_end ) continue;

									dir = search_range_start + j * search_dir_size;
									clearance = r_point * sin( theta_point - dir );

									if( clearance > 0 )	clearance = clearance - node_config.vehicle_width_left.value - node_config.clearance_required.value;
									else 				clearance = -(clearance + node_config.vehicle_width_right.value + node_config.clearance_required.value);
									clearance = clearance > 0 ? clearance : 0;

									// update
									if( eval_trajectories[j].clearance > clearance ) {
										eval_trajectories[j].clearance = clearance;
									}
									if( clearance <= 0 ) {
										eval_trajectories[j].clearance = 0;
										eval_trajectories[j].depth = 0;
									}
								}

							} // <--- evaluate the clearance

							i_max_eval = -1;
							max_eval = 0;
							for( int i = 0; i < (signed)eval_trajectories.size(); i++ ) {
								double eval;
								double eval_clearance;
								double eval_dir;
								double dir;
								dir = search_range_start + i * search_dir_size;

								// not have required clearance
								if( eval_trajectories[i].clearance <= 0 ) continue;
								if( eval_trajectories[i].depth <= 0 ) continue;

								eval_clearance = eval_trajectories[i].clearance / node_config.clearance_margin.value;
								eval_dir = cos(eval_trajectories[i].dir);
								eval_dir = eval_dir > 0 ? eval_dir : 0;
								eval = node_config.mediate_trajectory_weight_clearance.value * eval_clearance
									+ node_config.mediate_trajectory_weight_direction.value * eval_dir;

								if( max_eval < eval ) {
									max_eval = eval;
									i_max_eval = i;
								}
							}
							var_debug_double = eval_trajectories.size();


							// ---> set actual trajectory
							if( i_max_eval >= 0 ) {
								gnd::matrix::fixed<4,4>  		coord_tf_matrix_pathend2global;
								gnd::vector::fixed_column<4>	trajectory_on_pathend;
								gnd::vector::fixed_column<4>	trajectory_on_global;

								trajectory_on_pathend[0] = trajectories_searched_on_pathend[i_max_eval].end.x;
								trajectory_on_pathend[1] = trajectories_searched_on_pathend[i_max_eval].end.y;
								trajectory_on_pathend[2] = 0;
								trajectory_on_pathend[3] = 1;

								coordinate_tree.get_convert_matrix(coordinate_id_pathend, coordinate_id_global, &coord_tf_matrix_pathend2global);
								gnd::matrix::prod(&coord_tf_matrix_pathend2global, &trajectory_on_pathend, &trajectory_on_global);

								trajectory_actual.lines.resize(1);
								trajectory_actual.lines[0].end.x = trajectory_on_global[0];
								trajectory_actual.lines[0].end.y = trajectory_on_global[1];
								trajectory_actual.lines[0].end.theta = trajectories_searched_on_pathend[i_max_eval].end.theta + msg_planned_path.path[0].end.theta;
								trajectory_actual.lines[0].curvature = trajectories_searched_on_pathend[i_max_eval].curvature;

								eval_trajectory_actual_for_velocity_determination = eval_trajectories[i_max_eval];
							} // <--- set actual trajectory
							schedule_planning_trajecotry_actual = gnd_loop_next( time_current, time_start, 3.0 );
						} // <--- if robot is not near the target trajectory, the actual trajectory is planned an intermediate trajectory
						// ---> if robot is near the target trajectory, the actual trajectory is the target trajectory
						else {
							// the actual trajectory is the target trajectory
							trajectory_actual.lines.resize(1);
							trajectory_actual.lines[0] = trajectory_target.lines[0];

							eval_trajectory_actual_for_velocity_determination = eval_trajectory_target_for_velocity_determination;
						} // <--- if robot is near the target trajectory, the actual trajectory is the target trajectory


						// ---> update coordinate
						if( trajectory_actual.lines.size() > 0 ){
							gnd::matrix::fixed<4,4>  		coord_tf_matrix_actual2global;

							gnd::matrix::coordinate_converter(&coord_tf_matrix_actual2global,
									trajectory_actual.lines[0].end.x, trajectory_actual.lines[0].end.y, 0,
									cos(trajectory_actual.lines[0].end.theta), sin(trajectory_actual.lines[0].end.theta), 0,
									0, 0, 1);
							coordinate_tree.set_coordinate(coordinate_id_trajectory_actual, &coord_tf_matrix_actual2global);
						} // <--- update coordinate

					} // <---  determine the actual trajectory


				} // <--- trajectory determination

		// 速度決定
			// 距離に基づく速度
			// 方向に基づく速度
			// 障害物までの距離に基づく速度
				// ---> velocity determination
				if( trajectory_actual.lines.size() <= 0 ) {
					velocity = 0;
				}
				else {
					double vel_dist_companion;
					double vel_dir;
					double vel_trajectory_depth;
					double vel_motion_depth;


					{ // ---> velocity designed based on distance to companion
						// the case
						if( node_config.topic_name_pose_companion.value[0] && !(flgs_errorstate & Flags_ErrorState_LostCompanion) ) {
							double dist = companion_on_pathend[0] - robot_on_pathend[0];
							var_debug_double = dist;
							if( dist < -node_config.distance_to_wait_for_companion.value ) {
								vel_dist_companion = 0.0;
							}
							else if( dist < -node_config.distance_to_slowdown_for_companion.value && prev_velocity > 0.0) {
								vel_dist_companion = (dist - (-node_config.distance_to_slowdown_for_companion.value)) / ((-node_config.distance_to_wait_for_companion.value) - (-node_config.distance_to_slowdown_for_companion.value)) * node_config.max_velocity.value;
							}
							else if( dist < -node_config.distance_to_slowdown_for_companion.value && prev_velocity == 0.0) {
								vel_dist_companion = 0.0;
							}
							else {
								vel_dist_companion = node_config.max_velocity.value;
							}
						}
						else if( node_config.topic_name_pose_companion.value[0] && (flgs_errorstate & Flags_ErrorState_LostCompanion) ) {
							vel_dist_companion = 0;
						}
						// not subscribe companion topic
						else {
							vel_dist_companion = node_config.max_velocity.value;
						}
						vel_dist_companion = vel_dist_companion < prev_velocity + node_config.acceleration.value ? vel_dist_companion : prev_velocity + node_config.acceleration.value;
						vel_dist_companion = vel_dist_companion > prev_velocity - node_config.deceleration.value ? vel_dist_companion : prev_velocity - node_config.deceleration.value;
					} // <--- velocity designed based on distance to companion


					{ // ---> velocity designed based on direction
						double dir_diff;
						dir_diff = gnd_rad_normalize(trajectory_actual.lines[0].end.theta - msg_pose.theta);

						vel_dir = node_config.max_velocity.value * 2.0 * (cos(dir_diff) - 1.0 /2.0);
						//vel_dir = node_config.max_velocity.value * 2.0 * (cos(dir_diff) + 1/6);
						//if(cos(dir_diff) < 1.0/2.0 && cos(dir_diff) > -1/6){
						//	vel_dir = 0.2 * 2.0 * (cos(dir_diff) +1/6);
						//}	
						vel_dir = vel_dir < 0 ? 0 : vel_dir;

						vel_dir = vel_dir < prev_velocity + node_config.acceleration.value ? vel_dir : prev_velocity + node_config.acceleration.value;
						vel_dir = vel_dir > prev_velocity - node_config.deceleration.value ? vel_dir : prev_velocity - node_config.deceleration.value;
					} // <--- velocity designed based on direction


					{ // ---> velocity designed based on depth
						if( eval_trajectory_actual_for_velocity_determination.depth >= node_config.depth_for_slow_down.value ) {
							vel_trajectory_depth = node_config.max_velocity.value;
							//Spur_set_accel(0.3);
						}
						else if( eval_trajectory_actual_for_velocity_determination.depth > node_config.depth_for_stop.value ) {
							vel_trajectory_depth = node_config.max_velocity.value *
									( (eval_trajectory_actual_for_velocity_determination.depth - node_config.depth_for_stop.value) / (node_config.depth_for_slow_down.value - node_config.depth_for_stop.value));
							//Spur_set_accel(0.3);
						}
						else {
							vel_trajectory_depth = 0;
							//Spur_set_accel(0.3);
						}
					} // <--- velocity designed based on depth


					{ // ---> velocity designed based on depth
						if( obstalce_for_motion.depth - node_config.clearance_required.value >= node_config.depth_for_slow_down.value ) {
							vel_motion_depth = node_config.max_velocity.value;
						}
						else {
							vel_motion_depth = node_config.max_velocity.value *
									( (obstalce_for_motion.depth - node_config.clearance_required.value) / (node_config.depth_for_slow_down.value));
						}
					} // <--- velocity designed based on depth

					velocity = vel_trajectory_depth < vel_motion_depth ? vel_trajectory_depth : vel_motion_depth;
					velocity = velocity < vel_dir ? velocity : vel_dir;
					velocity = velocity < vel_dist_companion ? velocity : vel_dist_companion;

				} // <--- velocity determination


		// 軌跡に従うための制御コマンドの送信
			// 決定された速度と軌道に基づいて、ロボットが実際にどのように動作するかを制御
			// 障害物が検出された場合や自律モードでない場合は停止するなどの指示

				{ // ---> send control command to follow the trajectory
					// set default command (stop command)
					//jwvehicle::motionctrl_setcmd_Stop(&msg_vehicle_ctrl);

					// error state case
					if( flgs_errorstate ) {
						// send stop command
						//jwvehicle::motionctrl_setcmd_Stop(&msg_vehicle_ctrl);
						Spur_stop();
						prev_velocity = 0.0;
					}
					// not Auto Mode case
					else if( node_config.topic_name_mode_status.value[0] && !flagAutoMode ) {
						
						// don't create Tracking node case.
						//[note] if Tracking node is created, comment out this spur command.
						if(msg_mode_status.status == msg_mode_status.MODE_STATE_TRACKING)  Spur_stop();
						
						prev_velocity = 0.0;
					}
					// chenge Manual mood
					else if ( node_config.topic_name_controller.value[0] && flagManual ) {
						Spur_free();
					}
					// no trajectory case
					else if( trajectory_actual.lines.size() <= 0 ) {
						// send stop command
						//jwvehicle::motionctrl_setcmd_Stop(&msg_vehicle_ctrl);
						Spur_stop();
						prev_velocity = 0.0;
					}
					else if( trajectory_actual.lines.size() > 0 ) {
						// send trajectory and velocity limitation
						// stop on line command
						//jwvehicle::motionctrl_setcmd_StopOnLineFL_GL(&msg_vehicle_ctrl,
						//		velocity, node_config.max_angular_velocity.value,
						//		trajectory_actual.lines[0].end.x, trajectory_actual.lines[0].end.y, trajectory_actual.lines[0].end.theta );
						if ( !node_config.topic_name_controller.value[0] ||
								(node_config.topic_name_controller.value[0] && flagMovable) ) {
							/* disabled remote or (enabled and pushed the button) */

							Spur_set_vel(std::min({velocity, node_config.max_velocity.value, external_controlled_velocity}));
							//if ( velocity > node_config.max_velocity.value ) {
							//	Spur_set_vel( node_config.max_velocity.value );
							//} else {
							//	Spur_set_vel(velocity);
							//}
							Spur_set_angvel(node_config.max_angular_velocity.value);

							Spur_stop_line_GL( trajectory_actual.lines[0].end.x,trajectory_actual.lines[0].end.y,trajectory_actual.lines[0].end.theta );

							prev_velocity = velocity;
						}
						else {
							Spur_stop();
							prev_velocity = 0.0;
						}
					}
					else {
						// impossible case
						// set default command (stop command)
						//jwvehicle::motionctrl_setcmd_Stop(&msg_vehicle_ctrl);
						prev_velocity = 0.0;
					}

					// publish
					//pub_vehicle_ctrl.publish(msg_vehicle_ctrl);
				} // <--- send control command to follow the trajectory

		// ステータスの公開
			// ロボットの現在の状態（動作中、停止中など）をROSトピックを通じて公開
			// ロボットの状態（例：停止中、手動モード中など）が含まれる

				{ // ---> publish status
					if ( msg_planned_path.path.size() <= 0 ){
					//if ( trajectory_actual.lines.size() <= 0 && trajectory_target.lines.size() <= 0 ) {
						msg_vehicle_status.status = msg_vehicle_status.VEHICLE_STATE_IDLE;
					} else {
						if ( node_config.topic_name_controller.value[0] && !flagMovable ) {
							msg_vehicle_status.status = msg_vehicle_status.VEHICLE_STATE_STOP_REMOTE;
						} else if ( node_config.topic_name_controller.value[0] && flagManual ){ 
							msg_vehicle_status.status = msg_vehicle_status.VEHICLE_STATE_STOP_MANUAL; 
						} else if ( eval_trajectory_actual_for_velocity_determination.depth < node_config.depth_for_stop.value
								&& std::fabs(velocity) < DBL_EPSILON ) {
							msg_vehicle_status.status = msg_vehicle_status.VEHICLE_STATE_STOP_OBSTACLE;
						} else {
							msg_vehicle_status.status = msg_vehicle_status.VEHICLE_STATE_RUN;
						}
					}

					pub_vehicle_stauts.publish(msg_vehicle_status);
				} // <--- publish status

				// schedule next planning
				schedule_planning = gnd_loop_next( time_current, time_start, node_config.period_planning.value );
			} // <---  planning

	// 計画された軌道の公開
		// 計画された軌道やターゲット軌道、実際の軌道を定期的に公開し、他のノードやシステムがこれらの情報を使用できるようにする

			// ---> publish planed trajectory
			if( time_current > schedule_publish_planning ) {
				// planned path
				if( msg_planned_path.path.size() > 0 ) {
					pub_planned_path.publish(msg_planned_path);
				}
				// target trajectory
				if( trajectory_target.lines.size() > 0 ) {
					msg_trajectory_target.end.x = trajectory_target.lines[0].end.x;
					msg_trajectory_target.end.y = trajectory_target.lines[0].end.y;
					msg_trajectory_target.end.theta = trajectory_target.lines[0].end.theta;
					msg_trajectory_target.curvature = trajectory_target.lines[0].curvature;
					// publish
					pub_trajectory_target.publish(msg_trajectory_target);
				}
				// actual trajectory
				if( trajectory_actual.lines.size() > 0 ) {
					msg_trajectory_actual.end.x = trajectory_actual.lines[0].end.x;
					msg_trajectory_actual.end.y = trajectory_actual.lines[0].end.y;
					msg_trajectory_actual.end.theta = trajectory_actual.lines[0].end.theta;
					msg_trajectory_actual.curvature = trajectory_actual.lines[0].curvature;
					// publish
					pub_trajectory_actual.publish(msg_trajectory_actual);
				}

				// next schedule
				schedule_publish_planning = gnd_loop_next(time_current, time_start, node_config.period_publish_planning.value);
			} // ---> publish planed trajectory

			// ---> status display
			if(node_config.period_cui_status_display.value > 0 && time_current > schedule_display ) {
				if( nline_display ) {
					::fprintf(stderr, "\x1b[%02dA", nline_display);
					nline_display = 0;
				}

				nline_display++; ::fprintf(stderr, "\x1b[K-------------------- \x1b[1m\x1b[36m%s\x1b[39m\x1b[0m --------------------\n", node_config.node_name.value);
				nline_display++; ::fprintf(stderr, "\x1b[K   operating time : %6.01lf[sec]\n", time_current - time_start);


				if( msg_planned_path.path.size() > 0) {
					end_path_id = stoi(msg_planned_path.path[0].end.name);
					nline_display++; ::fprintf(stderr, "\x1b[Krobot on path-end : %.3lf, %.3lf,  %.3lf\n", robot_on_pathend[0], robot_on_pathend[1], theta_robot_on_pathend);
					nline_display++; ::fprintf(stderr, "\x1b[K current path-end : %.3f, %.3lf,  %.3lf\n", msg_planned_path.path[0].end.x, msg_planned_path.path[0].end.y, 180.0 * msg_planned_path.path[0].end.theta / M_PI );
					nline_display++; ::fprintf(stderr, "\x1b[K current path-end-id: %d\n,", end_path_id);
				}
				else {
					nline_display++; ::fprintf(stderr, "\x1b[Krobot on path-end : ---\n");
					nline_display++; ::fprintf(stderr, "\x1b[K current path-end : ---\n");
				}

				if( trajectory_target.lines.size() > 0) {
					nline_display++; ::fprintf(stderr, "\x1b[Ktarget trajectory : %.3lf, %.3lf,  %.3lf\n", trajectory_target.lines[0].end.x, trajectory_target.lines[0].end.y, 180.0 * trajectory_target.lines[0].end.theta / M_PI );
				}
				else {
					nline_display++; ::fprintf(stderr, "\x1b[Ktarget trajectory : ---\n" );
				}

				if( trajectory_target.lines.size() > 0) {
					nline_display++; ::fprintf(stderr, "\x1b[Kactual trajectory : %.3lf, %.3lf, %.3lf\n", trajectory_actual.lines[0].end.x, trajectory_actual.lines[0].end.y, 180.0 * trajectory_actual.lines[0].end.theta / M_PI );
					nline_display++; ::fprintf(stderr, "\x1b[K                  : depth %.3lf, cl %.3lf, dir %.3lf\n", eval_trajectory_actual_for_velocity_determination.depth, eval_trajectory_actual_for_velocity_determination.clearance, eval_trajectory_actual_for_velocity_determination.dir);
				}
				else {
					nline_display++; ::fprintf(stderr, "\x1b[Kactual trajectory : ---\n" );
					nline_display++; ::fprintf(stderr, "\x1b[K                  : ---\n" );
				}

				if( node_config.topic_name_pose_companion.value[0] ) {
					nline_display++; ::fprintf(stderr, "\x1b[K        companion : %.3lf, %.3lf, %.3lf\n", msg_pose_companion.x, msg_pose_companion.y, gnd_rad2deg(msg_pose_companion.theta) );
				}
				nline_display++; ::fprintf(stderr, "\x1b[Kschedule reselect : %lf[sec]\n", schedule_reselect_trajecotry - time_current);
				nline_display++; ::fprintf(stderr, "\x1b[K distance to obst : %lf[m]\n", eval_trajectory_target_for_velocity_determination.depth);
				nline_display++; ::fprintf(stderr, "\x1b[K     search range : %lf %lf[m]\n", search_range_right, search_range_left );
				nline_display++; ::fprintf(stderr, "\x1b[K      error state : 0x%x[sec]\n", flgs_errorstate);
				nline_display++; ::fprintf(stderr, "\x1b[K            debug : %6.03lf\n", var_debug_double);

				nline_display++; ::fprintf(stderr, "\x1b[K       controller : %s\n", node_config.topic_name_controller.value[0]? "enabled": "disabled");
				if( node_config.topic_name_mode_status.value[0]) {
					nline_display++; ::fprintf(stderr, "\x1b[K        Auto mode : %s\n", flagAutoMode? "true": "false");
				}
				if (node_config.topic_name_controller.value[0]) {
					nline_display++; ::fprintf(stderr, "\x1b[K          movable : %s\n", flagMovable? "true": "false");
					nline_display++; ::fprintf(stderr, "\x1b[K           manual : %s\n", flagManual? "true": "false");
					nline_display++; ::fprintf(stderr, "\x1b[K         watchdog : %lf[sec]\n", node_config.time_watchdog_reference.value );
				}
				//if (node_config.topic_name_vehicle_status.value[0]) {
				//	nline_display++; ::fprintf(stderr, "\x1b[K   vehicle status : %s\n", msg_vehicle_status.status );
				//}

				schedule_display = gnd_loop_next(time_current, time_start, node_config.period_cui_status_display.value);
				} // <--- check formation mode
			} // <--- status display
		} // <--- main loop
	} // <--- operate


	{ // ---> finalize
		fprintf(stdout, "---------- finalize ----------\n");

		{ // ---> order to stop
			//jwvehicle::motionctrl_setcmd_Stop(&msg_vehicle_ctrl);
			// publish
			//pub_vehicle_ctrl.publish(msg_vehicle_ctrl);
			Spur_stop();
			//Spur_free();

		} // <--- order to stop
		fprintf(stderr, "  ... %s fin\n", node_config.node_name.value);
	} // <--- finalize

	return 0;
}
