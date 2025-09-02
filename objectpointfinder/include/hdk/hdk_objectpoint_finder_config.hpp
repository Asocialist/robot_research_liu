/*
 * @file hdk_pose_evaluator_config.hpp
 * @author Hidekazu TAKAHASHI
 * @date 2017/10/25
 */

#ifndef HDK_OBJECTPOINT_FINDER_CONFIG_HPP_
#define HDK_OBJECTPOINT_FINDER_CONFIG_HPP_

#include <string.h>

#include "gnd/gnd-config-file.hpp"
#include "gnd/gnd-lib-error.h"
#include "gnd/gnd-util.h"



// ---> type declaration
namespace hdk {
	namespace objectpoint_finder {
		struct node_config;
		typedef struct node_config node_config_t;

		typedef gnd::conf::parameter_array<char, 256>	param_string_t;
		typedef gnd::conf::parameter_array<double, 3>	param_geo3d_t;
		typedef gnd::conf::parameter_array<int, 2>		param_image_size_t;
		typedef gnd::conf::param_int					param_int_t;
		typedef gnd::conf::param_long					param_long_t;
		typedef gnd::conf::param_double					param_double_t;
		typedef gnd::conf::param_bool					param_bool_t;
	}
} // <--- type declaration


// ---> const variables definition
namespace hdk {
	namespace objectpoint_finder {

		// ---> ros commnication
		static const param_string_t Default_node_name = {
				"node-name",
				"objectpoint_finder",
				"ros-node name"
		};

		static const param_string_t Default_topic_name_pose = {
				"topic-name-pose",
				"pose",
				"pose topic name, (subscribe, type:gnd_geometry2d_msgs/msg_pose2d_stamped)"
		};

		static const param_string_t Default_service_name_find_objectpoint = {
				"service-name-find-objectpoint",
				"find_objectpoint",
				"service name to find objectpoint (server)"
		};

		static const param_string_t Default_service_name_is_in_travelable_area = {
				"service-name-is-in-travelable-area",
				"is_in_travelable_area",
				"service name to check specified position is in travelable area or not (server)"
		};
		// <--- ros commnication


		// ---> path file option
		static const param_string_t Default_path_map_file = {
				"path-map-file",
				"",
				"path map file name"
		};
		// <--- path file option


		// ---> debug option
		static const param_double_t Default_period_cui_status_display = {
				"period-cui-status-display",
				0.0,
				"display the node status in terminal. [note] it need ansi color code"
		};
		// <--- debug option
	}
}
// <--- const variables definition


// ---> function declaration
namespace hdk {
	namespace objectpoint_finder {

		/**
		 * @brief initialize configure to default parameter
		 * @param [out] p : node_config
		 */
		int init_node_config(node_config *conf);


		/**
		 * @brief config file read
		 * @param [in] fname : file name
		 * @param [out] dest : configuration parameter
		 */
		int fread_node_config( const char* fname, node_config *dest );
		/**
		 * @brief get config parameter from description
		 * @param [out] dest : configuration parameter
		 * @param [in]  src  : configuration description
		 */
		int get_node_config( node_config *dest, gnd::conf::configuration *src );



		/**
		 * @brief config file write
		 * @param [in] fname : file name
		 * @param [in] src   : configuration parameter
		 */
		int fwrite_node_config( const char* fname, node_config *src );
		/**
		 * @brief set config description
		 * @param [out] dest : description
		 * @param [in]   src : parameter
		 */
		int set_node_config( gnd::conf::configuration *dest, node_config *src );
	}
} // <--- function declaration



// ---> type definition
namespace hdk {
	namespace objectpoint_finder {
		/**
		 * @brief configuration parameter for gnd_urg_proxy node
		 */
		struct node_config {
			node_config();

			// ros communication
			param_string_t					node_name;								///< node name
			param_string_t					topic_name_pose;						///< topic name(subscribe)
			param_string_t					service_name_find_objectpoint;				///< service name(server)
			param_string_t					service_name_is_in_travelable_area;		///< service name(server)


			// path file option
			param_string_t					path_map_file;					///< path map file

			// debug option
			param_double_t					period_cui_status_display;		///< cui status display mode
		};

		inline
		node_config::node_config() {
			init_node_config(this);
		}
		// <--- struct node_config
	}
}
// <--- type definition


// ---> function definition
namespace hdk {
	namespace objectpoint_finder {
		/*
		 * @brief initialize configuration parameter
		 * @param [out] p : node_config
		 */
		inline
		int init_node_config( node_config *p ){
			gnd_assert(!p, -1, "invalid null pointer argument\n" );

			// ros communication parameter
			memcpy( &p->node_name,							&Default_node_name,								sizeof(Default_node_name) );
			memcpy( &p->topic_name_pose,					&Default_topic_name_pose,						sizeof(Default_topic_name_pose) );
			memcpy( &p->service_name_find_objectpoint,			&Default_service_name_find_objectpoint,			sizeof(Default_service_name_find_objectpoint) );
			memcpy( &p->service_name_is_in_travelable_area,	&Default_service_name_is_in_travelable_area,	sizeof(Default_service_name_is_in_travelable_area) );
			// path file option
			memcpy( &p->path_map_file,						&Default_path_map_file,							sizeof(Default_path_map_file) );
			// debug option
			memcpy( &p->period_cui_status_display,			&Default_period_cui_status_display,				sizeof(Default_period_cui_status_display) );

			return 0;
		}

		/*
		 * @brief config file read
		 * @param [in] fname : file name
		 * @param [out] dest : configuration parameter
		 */
		inline
		int fread_node_config( const char* fname, node_config *dest ) {
			gnd_assert(!fname, -1, "invalid null pointer argument\n" );
			gnd_assert(!dest, -1, "invalid null pointer argument\n" );

			{ // ---> operation
				int ret;
				gnd::conf::file_stream fs;
				// configuration file read
				if( (ret = fs.read(fname)) < 0 )    return ret;

				return get_node_config(dest, &fs);
			} // <--- operation
		}
		/*
		 * @brief get config parameter from description
		 * @param [out] dest : configuration parameter
		 * @param [in]  src  : configuration description
		 */
		inline
		int get_node_config( node_config *dest, gnd::conf::configuration *src ) {
			gnd_assert(!dest, -1, "invalid null pointer argument\n" );
			gnd_assert(!src, -1, "invalid null pointer argument\n" );

			// ros communication parameter
			gnd::conf::get_parameter( src, &dest->node_name );
			gnd::conf::get_parameter( src, &dest->topic_name_pose );
			gnd::conf::get_parameter( src, &dest->service_name_find_objectpoint );
			gnd::conf::get_parameter( src, &dest->service_name_is_in_travelable_area );
			// map file option
			gnd::conf::get_parameter( src, &dest->path_map_file );
			// debug option
			gnd::conf::get_parameter( src, &dest->period_cui_status_display );

			return 0;
		}



		/*
		 * @brief config file write
		 * @param [in] fname : file name
		 * @param [in] src   : configuration parameter
		 */
		inline
		int fwrite_node_config( const char* fname, node_config *src ) {
			gnd_assert(!fname, -1, "invalid null pointer argument\n" );
			gnd_assert(!src, -1, "invalid null pointer argument\n" );
			{ // ---> operation
				int ret;
				gnd::conf::file_stream fs;
				// convert configuration declaration
				if( (ret = set_node_config(&fs, src)) < 0 ) return ret;

				return fs.write(fname);
			} // <--- operation
		}

		/*
		 * @brief set config description
		 * @param [out] dest : description
		 * @param [in]   src : parameter
		 */
		inline
		int set_node_config( gnd::conf::configuration *dest, node_config *src ) {
			gnd_assert(!dest, -1, "invalid null pointer argument\n" );
			gnd_assert(!src, -1, "invalid null pointer argument\n" );

			// ros communication parameter
			gnd::conf::set_parameter( dest, &src->node_name );
			gnd::conf::set_parameter( dest, &src->topic_name_pose );
			gnd::conf::set_parameter( dest, &src->service_name_find_objectpoint );
			gnd::conf::set_parameter( dest, &src->service_name_is_in_travelable_area );
			// map file option
			gnd::conf::set_parameter( dest, &src->path_map_file );
			// debug option
			gnd::conf::set_parameter( dest, &src->period_cui_status_display );

			return 0;
		}
	}
}
// <--- function definition

#endif /* HDK_OBJECTPOINT_FINDER_CONFIG_HPP_ */
