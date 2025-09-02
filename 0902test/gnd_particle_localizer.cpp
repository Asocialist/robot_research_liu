/**
 * @file gnd_particle_localizer/src/main.cpp
 *
 * @brief Test Program
 **/

#include "gnd/gnd-multi-platform.h"
#include "gnd/gnd-multi-math.h"
#include "gnd/gnd_particle_localizer.hpp"

#include <stdio.h>
#include "ros/ros.h"

#include "gnd/gnd-random.hpp"
#include "gnd/gnd-util.h"

#include "gnd_synctimer/srv_get_synctime.h"

typedef gnd::particle_localizer::msg_pose_t 										msg_pose_t;
typedef gnd::particle_localizer::msg_motion_t										msg_motion_t;
typedef gnd::particle_localizer::msgreader_motion_t									msgreader_motion_t;
typedef gnd::particle_localizer::msg_particles_t									msg_particles_t;
typedef gnd::particle_localizer::particles_t										particles_t;
typedef gnd::particle_localizer::particle_t											particle_t;
typedef gnd::particle_localizer::systematic_error_ratio_t							particle_systematic_error_ratio_t;
typedef std::vector<gnd::particle_localizer::systematic_error_ratio_t>				particle_systematic_error_ratios_t;
typedef gnd::particle_localizer::msg_particle_weights_t								msg_particle_weights_t;
typedef gnd::particle_localizer::msgreader_particle_weights_t						msgreader_particle_weights_t;
typedef gnd::particle_localizer::particle_weights_t									particle_weights_t;
typedef gnd::particle_localizer::particle_weight_t									particle_weight_t;

typedef gnd::particle_localizer::srv_reset_particles_nd_request_t					srv_reset_particles_nd_request_t;
typedef gnd::particle_localizer::srv_reset_particles_nd_response_t					srv_reset_particles_nd_response_t;
typedef gnd::particle_localizer::srv_funcobj_reset_particles_normal_distribution	srv_funcobj_reset_particles_nd_t;

typedef gnd_synctimer::srv_get_synctime												srv_get_synctime_t;

int main(int argc, char **argv) {

	gnd::particle_localizer::node_config			node_config;

	{ // ---> start up, read configuration file
		if( argc > 1 ) {
			if( gnd::particle_localizer::fread_node_config( argv[1], &node_config ) < 0 ){
				char fname[1024];
				fprintf(stdout, "   ... Error: fail to read config file \"%s\"\n", argv[1]);
				sprintf(fname, "%s.tmp", argv[1]);
				// file out configuration file
				if( gnd::particle_localizer::fwrite_node_config( fname, &node_config ) >= 0 ){
					fprintf(stdout, "            : output sample configuration file \"%s\"\n", fname);
				}
				return -1;
			}
			else {
				fprintf(stdout, "   ... read config file \"%s\"\n", argv[1]);
			}
		}
	} // <--- start up, read configuration file



	{ // ---> initialize rosÅ@platform
		if( node_config.node_name.value[0] ) {
			ros::init(argc, argv, node_config.node_name.value);
		}
		else {
			fprintf(stdout, "   ... Error: node name is null, you must specify the name of this node via config item \"%s\"\n", node_config.node_name.item);
			return -1;
		}
		fprintf(stdout, " node: \"%s\"\n", node_config.node_name.value);
	} // <--- initialize rosÅ@platform

	// ros nodehandle
	ros::NodeHandle						nodehandle;						// node handle
	ros::NodeHandle						nodehandle_private("~");		// private node handle

	ros::Publisher						pub_pose;						// estimated pose message publisher
	msg_pose_t							msg_pose;						// estimated pose message

	ros::Subscriber						subsc_motion;					// motion message subscriber
	msgreader_motion_t					msgreader_motion;				// motion message reader

	ros::Publisher						pub_particles;					// particles message publisher
	msg_particles_t						msg_particles;					// particles message

	ros::Subscriber						subsc_particle_weights;			// particle weight message subscriber
	msgreader_particle_weights_t		msgreader_particle_weights;		// particle weight message reader

	ros::ServiceServer					srvserv_reset_particles_nd;		// reset particles normal distribution service server
	srv_funcobj_reset_particles_nd_t	srvfo_reset_particles_nd;		// reset particles normal distribution service function object

	particle_systematic_error_ratios_t systematic_error_ratio_depened_on_translate;

	ros::ServiceClient 					srvclnt_client_get_synctime;

	FILE *fp_txtlog = 0;

	{ // ---> initialization
		int phase = 0;
		ros::Time time_start;
		fprintf(stdout, "---------- initialization ----------\n");

		// ---> show initialize phase task
		if( ros::ok() ) {
			fprintf(stdout, " => initialization task\n");
			fprintf(stdout, "   %d. make estimated pose publisher\n", ++phase);
			fprintf(stdout, "   %d. make motion subscriber\n", ++phase);
			fprintf(stdout, "   %d. make particles publisher\n", ++phase);
			fprintf(stdout, "   %d. make particle weights subscriber\n", ++phase);
			fprintf(stdout, "   %d. make service server to reset particles\n", ++phase);
			if( node_config.service_name_get_synctimer.value[0] && node_config.period_time_synchronize.value > 0) {
				fprintf(stdout, "   %d. make service client for time synchronization\n", ++phase);
			}
			fprintf(stdout, "\n");
		} // <--- show initialize phase task

		phase = 0;
		// ---> make estimated pose publisher
		if( ros::ok() ) {
			fprintf(stdout, "\n");
			fprintf(stdout, " => %d. make estimated pose publisher\n", ++phase);

			if( !node_config.topic_name_pose.value[0] ) {
				ros::shutdown();
				fprintf(stdout, "    ... error: invalid topic name\n");
			}
			else {
				fprintf(stdout, "    ... topic name is \"%s\"\n", node_config.topic_name_pose.value );
				pub_pose = nodehandle.advertise<msg_pose_t>(node_config.topic_name_pose.value, 10 );

				msg_pose.header.stamp = time_start;
				msg_pose.header.seq = 0;
				msg_pose.header.frame_id = "";
				msg_pose.x = node_config.initial_pose.value[0];
				msg_pose.y = node_config.initial_pose.value[1];
				msg_pose.theta = node_config.initial_pose.value[2];

				fprintf(stdout, "    ... ok\n");
			}
		} // <--- make estimated pose publisher

		// ---> make motion subscriber
		if( ros::ok() ) {
			fprintf(stdout, "\n");
			fprintf(stdout, " => %d. make motion subscriber\n", ++phase);

			if( !node_config.topic_name_motion.value[0] ) {
				ros::shutdown();
				fprintf(stdout, "    ... error: invalid topic name\n");
			}
			else {
				fprintf(stdout, "    ... topic name is \"%s\"\n", node_config.topic_name_motion.value );
				// allocate buffer
				msgreader_motion.allocate(100);

				// subscribe
				subsc_motion = nodehandle.subscribe(node_config.topic_name_motion.value, 10,
						&msgreader_motion_t::rosmsg_read,
						msgreader_motion.reader_pointer() );
				fprintf(stdout, "    ... ok\n");
			}
		} // <--- make motion subscriber

		// ---> make particles publisher
		if( ros::ok() ) {
			fprintf(stdout, "\n");
			fprintf(stdout, " => %d. make estimated pose publisher\n", ++phase);

			if( !node_config.topic_name_particles.value[0] ) {
				ros::shutdown();
				fprintf(stdout, "    ... error: invalid topic name\n");
			}
			else {
				fprintf(stdout, "    ... topic name is \"%s\"\n", node_config.topic_name_particles.value );
				pub_particles = nodehandle.advertise<msg_particles_t>(node_config.topic_name_particles.value, 1 );

				msg_particles.header.stamp = time_start;
				msg_particles.header.seq = 0;
				msg_particles.header.frame_id = node_config.node_name.value;
				msg_particles.poses.resize( node_config.number_of_particles.value );
				msg_particles.schedule_resampling = time_start;

				// systematic error
				systematic_error_ratio_depened_on_translate.resize(node_config.number_of_particles.value);

				{ // ---> initialize particles
					msg_particles_t::_poses_type::value_type init_particle;
					gnd::matrix::fixed<3,3> cov;
					gnd::matrix::fixed<3,3> ws;
					gnd::vector::fixed_column<3> pos;

					for (int i = 0; i < node_config.number_of_particles.value; i++ ) {
						// set co-variance
						cov[0][0] = node_config.error_covariance_initial_pose.value[0];
						cov[0][1] = cov[1][0] = node_config.error_covariance_initial_pose.value[1];
						cov[0][2] = cov[2][0] = node_config.error_covariance_initial_pose.value[2];
						cov[1][1] = node_config.error_covariance_initial_pose.value[4];
						cov[1][2] = cov[2][1] = node_config.error_covariance_initial_pose.value[5];
						cov[2][2] = node_config.error_covariance_initial_pose.value[8];
						// create random value according to co-variance
						gnd::random_gaussian_mult(&cov, 3, &ws, &pos);
						// add random value
						init_particle.x = node_config.initial_pose.value[0] + pos[0];
						init_particle.y = node_config.initial_pose.value[1] + pos[1];
						init_particle.theta = node_config.initial_pose.value[2] + pos[2];
						// set
						msg_particles.poses[i] = init_particle;

						systematic_error_ratio_depened_on_translate[i][0] = 0;
						systematic_error_ratio_depened_on_translate[i][1] = 0;
						systematic_error_ratio_depened_on_translate[i][2] = 0;
					}
				} // <--- initialize particles

				fprintf(stdout, "    ... ok\n");
			}
		} // <--- make particles publisher

		// ---> make particle weights subscriber
		if( ros::ok() ) {
			fprintf(stdout, "\n");
			fprintf(stdout, " => %d. make particle weights subscriber\n", ++phase);

			if( !node_config.topic_name_particle_weights.value[0] ) {
				ros::shutdown();
				fprintf(stdout, "    ... error: invalid topic name\n");
			}
			else {
				fprintf(stdout, "    ... topic name is \"%s\"\n", node_config.topic_name_particle_weights.value );
				// allocate buffer
				msgreader_particle_weights.allocate(100);

				// subscribe
				subsc_particle_weights = nodehandle.subscribe(node_config.topic_name_particle_weights.value, 10,
						&msgreader_particle_weights_t::rosmsg_read,
						msgreader_particle_weights.reader_pointer() );
				fprintf(stdout, "    ... ok\n");
			}
		} // <--- make particle weights subscriber

		// ---> make service server
		if( ros::ok() ) {
			fprintf(stdout, "\n");
			fprintf(stdout, " => %d. make service server to reset particles\n", ++phase);

			if( !node_config.service_name_reset_particles_nd.value[0] ) {
				fprintf(stdout, "    ... error: invalid service name\n");
				ros::shutdown();
			}
			else {
				fprintf(stdout, "    ... service name is \"%s\"\n", node_config.service_name_reset_particles_nd.value );

				// service server
				srvfo_reset_particles_nd.clear();
				srvserv_reset_particles_nd = nodehandle.advertiseService(
						node_config.service_name_reset_particles_nd.value,
						&srv_funcobj_reset_particles_nd_t::callback, &srvfo_reset_particles_nd);
				fprintf(stdout, "    ... ok\n");
			}
		} // <--- make service server


		// ---> make service client for time synchronization
		if( ros::ok() && node_config.service_name_get_synctimer.value[0] && node_config.period_time_synchronize.value > 0) {
			fprintf(stdout, "\n");
			fprintf(stdout, " => %d. create service client for time synchronization\n", ++phase );
			srvclnt_client_get_synctime = nodehandle.serviceClient<gnd_synctimer::srv_get_synctime>(node_config.service_name_get_synctimer.value);
		}// --->make service client for time synchronization


		// ---> text log file open
		if( ros::ok() && node_config.particles_log.value[0] ) {
			fprintf(stdout, "\n");
			fprintf(stdout, " => %d. create text log file\n", ++phase );
			fprintf(stdout, "    ... try to open \"%s\"\n", node_config.particles_log.value );

			if( !(fp_txtlog = fopen( node_config.particles_log.value, "w"))  ) {
				fprintf(stdout, "    ... error: fail to open \"%s\"\n", node_config.particles_log.value );
				ros::shutdown();
			}
			else {
				fprintf(fp_txtlog, "# [1. time] [2. index] [3. x] [4. y] [5. theta]\n" );
				fprintf(stdout, "    ... ok\n");
			}
		}
		// <--- text log file open

	} // <--- initialization



	if( ros::ok() ) { // ---> operation
		const double loop_freq = 1000;
		ros::Rate loop_rate(loop_freq);

		double time_offset = 0;
		double time_start = 0;
		double time_prev = 0;
		double time_current = 0;
		double schedule_resampling = 0;
		double schedule_timesync = 0;
		double time_display = 0;

		msg_motion_t msg_motion;
		msg_particle_weights_t particle_weights_integrated;		// likelihood ratio
		msg_particle_weights_t msg_particle_weights;			// likelihood

		bool flg_resampling = false;

		int cnt_weights_out_of_synchronization = 0;

		uint32_t seq_pose_prevdisplay = msg_pose.header.seq;
		uint32_t seq_motion_prevdisplay = msg_motion.header.seq;
		int cnt_particle_weights_display = 0;
		int cnt_resampling_display = 0;
		int nline_display = 0;

		int cnt_debug = 0;
		double var_debug_dbl = 0;

		{ // ---> initialize time variables
			time_offset = 0;
			time_start = ros::Time::now().toSec();
			time_prev = time_start;
			time_current = time_start;
			schedule_resampling = time_start;
			schedule_timesync = time_start;
			time_display = time_start;

			msg_particles.schedule_resampling.fromSec(schedule_resampling);
		} // <--- initialize time variables

		// initialize integrated particle weights storage
		particle_weights_integrated.weights.resize( msg_particles.poses.size(), (float) 1.0 / msg_particles.poses.size() );

		// ---> main loop
		fprintf(stderr, " => %s main loop start\n", node_config.node_name.value);
		while( ros::ok() ) {
			// blocking
			loop_rate.sleep();
			ros::spinOnce();

			// current time
			time_prev = time_current;
			time_current = ros::Time::now().toSec();
			if( time_current + time_offset < time_prev ) {
				time_current = time_prev + (1.0 / loop_freq) * 0.5;
			}
			else {
				time_current += time_offset;
			}

			if( srvfo_reset_particles_nd.is_called() ) {// ---> reset particles (service called)
				int size;

				fprintf(stdout, "reset particles\n");
				// set pose
				srvfo_reset_particles_nd.get_average( &msg_pose.x, &msg_pose.y, &msg_pose.theta );

				// resize
				srvfo_reset_particles_nd.get_size( &size );
				if( size > 0 ) {
					msg_particles.poses.resize( size );
				}

				{ // ---> initialize particles
					gnd::matrix::fixed<3,3> cov;
					gnd::matrix::fixed<3,3> ws;
					gnd::vector::fixed_column<3> rand;

					for (int i = 0; i < node_config.number_of_particles.value; i++ ) {
						// set co-variance
						srvfo_reset_particles_nd.get_covariance( gnd::matrix::pointer(&cov, 0, 0) );
						// create random value according to co-variance
						gnd::random_gaussian_mult(&cov, 3, &ws, &rand);

						// add random value
						msg_particles.poses[i].x = msg_pose.x + rand[0];
						msg_particles.poses[i].y = msg_pose.y + rand[1];
						msg_particles.poses[i].theta = gnd_rad_normalize( msg_pose.theta + rand[2] );
					}
				} // <--- initialize particles

				// publish(pose)
				msg_pose.header.stamp = msg_motion.header.stamp;
				msg_pose.header.seq++;
				pub_pose.publish(msg_pose);

				// publish(particles)
				msg_particles.header.stamp = msg_motion.header.stamp;
				msg_particles.header.seq++;
				pub_particles.publish(msg_particles);

				// weight reset
				particle_weights_integrated.weights.clear();
				particle_weights_integrated.weights.resize( msg_particles.poses.size(), (float) 1.0 / msg_particles.poses.size() );
				// have no evaluations
				flg_resampling = false;

				// clear
				srvfo_reset_particles_nd.clear();
				fprintf(stdout, "particle_reset\n");
			} // <--- reset particles (service called)


			// ---> particle transition with motion model
			if( msgreader_motion.copy_next( &msg_motion, msg_motion.header.seq ) == 0 ) {
				double trans_x, trans_y, rot;
				double cosv, sinv;

				cnt_debug++;

				trans_x = msg_motion.vel_x * msg_motion.measuring_period;
				trans_y = msg_motion.vel_y * msg_motion.measuring_period;
				rot = msg_motion.vel_ang * msg_motion.measuring_period;

				// pose
				cosv = cos(msg_pose.theta);
				sinv = sin(msg_pose.theta);
				msg_pose.x += cosv * trans_x - sinv * trans_y;
				msg_pose.y += sinv * trans_x + cosv * trans_y;
				msg_pose.theta += rot;

				{ // ---> particles transition
					gnd::matrix::fixed<3,3> error_covariance_vel;

					error_covariance_vel[0][0] = msg_motion.covariance[0];
					error_covariance_vel[0][1] = error_covariance_vel[1][0] = msg_motion.covariance[1];
					error_covariance_vel[0][2] = error_covariance_vel[2][0] = msg_motion.covariance[2];
					error_covariance_vel[1][1] = msg_motion.covariance[4];
					error_covariance_vel[1][2] = error_covariance_vel[2][1] = msg_motion.covariance[5];
					error_covariance_vel[2][2] = msg_motion.covariance[8];

					for( unsigned int i = 0; i < msg_particles.poses.size(); i++ ) {
						gnd::matrix::fixed<3,3> cp_error_cov, ws;
						gnd::vector::fixed_column<3> rand_err;
						gnd::vector::fixed_column<3> sys_err;

						// random error
						gnd::matrix::copy(&cp_error_cov, &error_covariance_vel);
						gnd::random_gaussian_mult(&cp_error_cov, 3, &ws, &rand_err);
						// systematic error
						gnd::matrix::scalar_prod( &systematic_error_ratio_depened_on_translate[i], msg_motion.vel_x, &sys_err );

						trans_x = ( msg_motion.vel_x + rand_err[0] + sys_err[0]) * msg_motion.measuring_period;
						trans_y = ( msg_motion.vel_y + rand_err[1] + sys_err[1] ) * msg_motion.measuring_period;
						rot = ( msg_motion.vel_ang + rand_err[2] + sys_err[2] ) * msg_motion.measuring_period;

						// pose calculation
						cosv = cos(msg_particles.poses[i].theta);
						sinv = sin(msg_particles.poses[i].theta);
						msg_particles.poses[i].x += cosv * trans_x - sinv * trans_y;
						msg_particles.poses[i].y += sinv * trans_x + cosv * trans_y;
						msg_particles.poses[i].theta += rot;

					}
				} // <--- particles transition


				// publish
				msg_pose.header.stamp = msg_motion.header.stamp;
				msg_pose.header.seq++;
				pub_pose.publish(msg_pose);

				msg_particles.header.stamp = msg_motion.header.stamp;
				msg_particles.header.seq++;
				pub_particles.publish(msg_particles);

				if( fp_txtlog ) {
					fprintf(fp_txtlog, "%lf %lf %lf %lf\n", time_current - time_start, msg_pose.x, msg_pose.y, msg_pose.theta);
					fflush(fp_txtlog);
				}
			} // <--- particle transition with motion model



			// ---> update particle weights by measurement model
			if( msgreader_particle_weights.copy_new(&msg_particle_weights, &msg_particle_weights.header.stamp) == 0 ) {
				cnt_particle_weights_display++;

				if( !gnd::rosutil::is_sequence_updated( particle_weights_integrated.seq_particles,  msg_particle_weights.seq_particles) ) {
					// out of synchronization
					cnt_weights_out_of_synchronization++;
				}
				else if( msg_particle_weights.weights.size() != particle_weights_integrated.weights.size() ) {
					// some particle weights are lacked
				}
				else {
					// integrate
					particle_weight_t sum = 0;
					for( unsigned int i = 0; i < particle_weights_integrated.weights.size(); i++ ){
						// integrate a particle weight
						particle_weights_integrated.weights[i] *= msg_particle_weights.weights[i];

						// zero weight exception
						if( particle_weights_integrated.weights[i] < gnd::particle_localizer::WEIGHT_T_EPSILON  ) {
							particle_weights_integrated.weights[i] = gnd::particle_localizer::WEIGHT_T_EPSILON;
						}
						// sum
						sum += particle_weights_integrated.weights[i];
					}

					// normalize (make the sum of weights to be equal 1.0) to avoid zero weight due to rounding error
					for( unsigned int i = 0; i < particle_weights_integrated.weights.size(); i++ ){
						particle_weights_integrated.weights[i] /= sum;
					}

					// resampling is possible
					flg_resampling = true;
				}

			} // <--- update particle weights by measurement model



			// ---> resampling according to particle weights
			if( time_current > schedule_resampling ) {
				// ---> resampling
				if( !flg_resampling ) {
					// particles are not evaluated
				}
				else {
					unsigned int i, j;
					particles_t copy_poses = msg_particles.poses;
					particle_systematic_error_ratios_t copy_syserr = systematic_error_ratio_depened_on_translate;
					particle_t  sum_poses;

					sum_poses.x = 0;
					sum_poses.y = 0;
					sum_poses.theta = 0;

					for( i = 0; i < msg_particles.poses.size(); i++ ) {
						double rand = gnd::random_uniform();	// the sum of weights is equal 1.0
						double sum = 0;

						// select a particle according to weight liner probability
						// scanning loop,
						for( j = 0; j < particle_weights_integrated.weights.size() - 1; j++ ) {
							sum += particle_weights_integrated.weights[j];
							if( sum > rand ) break;
						}

						{ // ---> set a new particle
							msg_particles.poses[i] = copy_poses[j];
							// sum
							sum_poses.x += msg_particles.poses[i].x;
							sum_poses.y += msg_particles.poses[i].y;
							sum_poses.theta += msg_particles.poses[i].theta;
						} // <--- set a new particle

						{ // ---> set systematic error ratio
							if( node_config.probability_change_systematic_motion_error.value < gnd::random_uniform() ) {
								// not change
								systematic_error_ratio_depened_on_translate[i] = copy_syserr[j];
							}
							else {
								// change
								systematic_error_ratio_depened_on_translate[i][0] = node_config.standard_systematic_motion_error.value[0] * gnd::random_gaussian(1.0);
								systematic_error_ratio_depened_on_translate[i][1] = node_config.standard_systematic_motion_error.value[1] * gnd::random_gaussian(1.0);
								systematic_error_ratio_depened_on_translate[i][2] = node_config.standard_systematic_motion_error.value[2] * gnd::random_gaussian(1.0);
							}
						} // <--- set systematic error ratio
					}

					// correct robot pose ( selected particles average )
					msg_pose.x = sum_poses.x / msg_particles.poses.size();
					msg_pose.y = sum_poses.y / msg_particles.poses.size();
					msg_pose.theta = sum_poses.theta / msg_particles.poses.size();

					// update sequence when resampling
					particle_weights_integrated.seq_particles = msg_particles.header.seq + 1;
					// reset weights
					particle_weights_integrated.weights.clear();
					particle_weights_integrated.weights.resize( msg_particles.poses.size(), (float) 1.0 / msg_particles.poses.size() );
					// have no evaluations
					flg_resampling = false;

					cnt_resampling_display++;
				} // <--- resampling

				// publish(pose)
				msg_pose.header.stamp = msg_motion.header.stamp;
				msg_pose.header.seq++;
				pub_pose.publish(msg_pose);

				// publish(particles)
				msg_particles.header.stamp = msg_motion.header.stamp;
				msg_particles.header.seq++;
				pub_particles.publish(msg_particles);

				// next schedule
				schedule_resampling = gnd_loop_next(time_current, time_start, node_config.period_resampling.value);
				msg_particles.schedule_resampling.fromSec(schedule_resampling);
			} // <--- resampling according to particle weights


			// ---> time synchronization
			if( node_config.service_name_get_synctimer.value[0] &&
				node_config.period_time_synchronize.value > 0 &&
				time_current > schedule_timesync ) {
				//
				gnd_synctimer::srv_get_synctime srv;
				ros::Time sndtime, rcvtime;

				sndtime = ros::Time::now();
				if( !srvclnt_client_get_synctime.call(srv) ){
					// error
				}
				else {
					double sndtime_dbl, rcvtime_dbl, selftime_dbl;
					double hosttime_dbl;
					double diff;
					// update
					rcvtime = ros::Time::now();

					sndtime_dbl = sndtime.toSec();
					rcvtime_dbl = rcvtime.toSec();
					selftime_dbl = (sndtime_dbl + rcvtime_dbl) / 2.0;
					hosttime_dbl = srv.response.time.data.toSec();

					diff = hosttime_dbl - selftime_dbl;
					if( fabs(diff) < 2.0) {
						time_offset = time_offset * 0.5 + (diff * 0.5);
					}
				}
				schedule_timesync = gnd_loop_next( time_current, time_start, node_config.period_time_synchronize.value );
				schedule_timesync += gnd::random_uniform() * node_config.period_time_synchronize.value;
			} // <--- time synchronization


			// ---> display status
			if( node_config.period_cui_status_display.value > 0
					&& time_current > time_display ) {

				if( nline_display ) {
					::fprintf(stderr, "\x1b[%02dA", nline_display);
					nline_display = 0;
				}

				nline_display++; ::fprintf(stderr, "\x1b[K-------------------- \x1b[1m\x1b[36m%s\x1b[39m\x1b[0m --------------------\n", node_config.node_name.value);
				nline_display++; ::fprintf(stderr, "\x1b[K operating time : %6.01lf[sec]\n", time_current - time_start);
				nline_display++; ::fprintf(stderr, "\x1b[K           time : %6.01lf[sec]\n", time_current);
				// position (publish)
				nline_display++; ::fprintf(stderr, "\x1b[K       position :   topic name \"%s\" (publish)\n", node_config.topic_name_pose.value);
				nline_display++; ::fprintf(stderr, "\x1b[K                :        value %8.03lf[m], %8.03lf[m], %6.01lf[deg]\n",
						msg_pose.x, msg_pose.y, gnd_ang2deg( msg_pose.theta ) );
				nline_display++; ::fprintf(stderr, "\x1b[K                :      publish %.01lf [Hz] %d\n",
						(double) (msg_pose.header.seq - seq_pose_prevdisplay) / node_config.period_cui_status_display.value, cnt_debug );
				seq_pose_prevdisplay = msg_pose.header.seq;
				// particles (publish)
				nline_display++; ::fprintf(stderr, "\x1b[K      particles : topic name \"%s\" (publish)\n", node_config.topic_name_particles.value );
				nline_display++; ::fprintf(stderr, "\x1b[K                : latest seq %d\n", msg_particles.header.seq );
				nline_display++; ::fprintf(stderr, "\x1b[K                : size %d\n", msg_particles.poses.size() );
				// velocity (subscribe)
				nline_display++; ::fprintf(stderr, "\x1b[K         motion :   topic name \"%s\" (subscribe)\n", node_config.topic_name_motion.value);
				nline_display++; ::fprintf(stderr, "\x1b[K                :        value %5.02lf[m/s], %4.02lf[m/s], %4.01lf[deg/s]\n",
						msg_motion.vel_x, msg_motion.vel_y, gnd_ang2deg( msg_motion.vel_ang ) );
				nline_display++; ::fprintf(stderr, "\x1b[K                : standard err %.02lf[m/s], %.02lf[m/s], %4.01lf[deg/s]\n",
						sqrt(msg_motion.covariance[0]), sqrt(msg_motion.covariance[4]), gnd_ang2deg( sqrt(msg_motion.covariance[8])) );
				nline_display++; ::fprintf(stderr, "\x1b[K                : latest seq %d\n", msg_motion.header.seq );
				// weight (subscribe)
				nline_display++; ::fprintf(stderr, "\x1b[K        weights :   topic name \"%s\" (subscribe)\n", node_config.topic_name_particle_weights.value);
				nline_display++; ::fprintf(stderr, "\x1b[K                :    subscribe %d [cnt]\n", cnt_particle_weights_display );
				// resampling
				nline_display++; ::fprintf(stderr, "\x1b[K     resampling : count %d, out of sync %d, period %.02lf\n", cnt_resampling_display, cnt_weights_out_of_synchronization, node_config.period_resampling.value );
				nline_display++; ::fprintf(stderr, "\x1b[K       synctime : offset %.04lf\n", time_offset );
				nline_display++; ::fprintf(stderr, "\x1b[K       debug : offset %.04lf\n", var_debug_dbl );

				nline_display++; ::fprintf(stderr, "\x1b[K\n");

				// update
				time_display = gnd_loop_next(time_current, time_start, node_config.period_cui_status_display.value );
			} // <--- display status

		} // <--- main loop
	} // <--- operation



	{ // ---> finalization
		fprintf(stderr, "\n");
		fprintf(stderr, "---------- finalization ----------\n");

		fprintf(stderr, " ... fin\n");
	} // <--- finalization
	return 0;
}
