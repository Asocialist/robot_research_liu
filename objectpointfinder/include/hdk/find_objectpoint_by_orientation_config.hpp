/*
 * @file find_objectpoint_by_orientation_config.hpp
 * @author Kosei SHINO
 * @date 2024/9/30
 */

#ifndef HDK_FIND_OBJECTPOINT_BY_ORIENTATION_CONFIG_HPP_
#define HDK_FIND_OBJECTPOINT_BY_ORIENTATION_CONFIG_HPP_

#include <string.h>

#include "gnd/gnd-config-file.hpp"
#include "gnd/gnd-lib-error.h"
#include "gnd/gnd-util.h"

// ---> type declaration
namespace hdk {
    namespace objectpoint_finder {
        struct node_config;
        typedef struct node_config node_config_t;

        typedef gnd::conf::parameter_array<char, 256>   param_string_t;
        typedef gnd::conf::parameter_array<double, 3>   param_geo3d_t;
        typedef gnd::conf::parameter_array<int, 2>      param_image_size_t;
        typedef gnd::conf::param_int                    param_int_t;
        typedef gnd::conf::param_long                   param_long_t;
        typedef gnd::conf::param_double                 param_double_t;
        typedef gnd::conf::param_bool                   param_bool_t;
    }
}

// ---> const variables definition
namespace hdk {
    namespace objectpoint_finder {
        static const param_string_t Default_node_name = {
            "node-name",
            "find_objectpoint_by_orientation",
            "ros-node name"
        };

        static const param_string_t Default_topic_name_pose = {
            "topic-name-pose",
            "pose",
            "pose topic name, (subscribe, type:gnd_geometry2d_msgs/msg_pose2d_stamped)"
        };

        static const param_string_t Default_topic_name_tracking_position = {
            "topic-name-tracking-position",
            "tracking_position",
            "tracking position topic name, (subscribe, type:geometry_msgs/Pose2D)"
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

        static const param_string_t Default_path_map_file = {
            "path-map-file",
            "",
            "path map file name"
        };

        static const param_double_t Default_period_cui_status_display = {
            "period-cui-status-display",
            0.0,
            "display the node status in terminal. [note] it need ansi color code"
        };

        static const param_bool_t Default_is_tracking_position_global = {
            "is-tracking-position-global",
            false,
            "true if tracking position is in global coordinates, false if in local coordinates"
        };

        static const param_double_t Default_max_distance = {
            "max-distance",
            10.0,
            "maximum distance to search for objectpoints"
        };

        static const param_double_t Default_angle_threshold = {
            "angle-threshold",
            M_PI/4,
            "angle threshold for objectpoint search"
        };
    }
}

// ---> function declaration
namespace hdk {
    namespace objectpoint_finder {
        int init_node_config(node_config *conf);
        int fread_node_config(const char* fname, node_config *dest);
        int get_node_config(node_config *dest, gnd::conf::configuration *src);
        int fwrite_node_config(const char* fname, node_config *src);
        int set_node_config(gnd::conf::configuration *dest, node_config *src);
    }
}

// ---> type definition
namespace hdk {
    namespace objectpoint_finder {
        struct node_config {
            node_config();

            param_string_t node_name;
            param_string_t topic_name_pose;
            param_string_t topic_name_tracking_position;
            param_string_t service_name_find_objectpoint;
            param_string_t service_name_is_in_travelable_area;
            param_string_t path_map_file;
            param_double_t period_cui_status_display;
            param_bool_t is_tracking_position_global;
            param_double_t max_distance;
            param_double_t angle_threshold;
        };

        inline
        node_config::node_config() {
            init_node_config(this);
        }
    }
}

// ---> function definition
namespace hdk {
    namespace objectpoint_finder {
        inline
        int init_node_config(node_config *p) {
            gnd_assert(!p, -1, "invalid null pointer argument\n");

            memcpy(&p->node_name, &Default_node_name, sizeof(Default_node_name));
            memcpy(&p->topic_name_pose, &Default_topic_name_pose, sizeof(Default_topic_name_pose));
            memcpy(&p->topic_name_tracking_position, &Default_topic_name_tracking_position, sizeof(Default_topic_name_tracking_position));
            memcpy(&p->service_name_find_objectpoint, &Default_service_name_find_objectpoint, sizeof(Default_service_name_find_objectpoint));
            memcpy(&p->service_name_is_in_travelable_area, &Default_service_name_is_in_travelable_area, sizeof(Default_service_name_is_in_travelable_area));
            memcpy(&p->path_map_file, &Default_path_map_file, sizeof(Default_path_map_file));
            memcpy(&p->period_cui_status_display, &Default_period_cui_status_display, sizeof(Default_period_cui_status_display));
            memcpy(&p->is_tracking_position_global, &Default_is_tracking_position_global, sizeof(Default_is_tracking_position_global));
            memcpy(&p->max_distance, &Default_max_distance, sizeof(Default_max_distance));
            memcpy(&p->angle_threshold, &Default_angle_threshold, sizeof(Default_angle_threshold));

            return 0;
        }

        inline
        int fread_node_config(const char* fname, node_config *dest) {
            gnd_assert(!fname, -1, "invalid null pointer argument\n");
            gnd_assert(!dest, -1, "invalid null pointer argument\n");

            {
                int ret;
                gnd::conf::file_stream fs;
                if ((ret = fs.read(fname)) < 0) return ret;
                return get_node_config(dest, &fs);
            }
        }

        inline
        int get_node_config(node_config *dest, gnd::conf::configuration *src) {
            gnd_assert(!dest, -1, "invalid null pointer argument\n");
            gnd_assert(!src, -1, "invalid null pointer argument\n");

            gnd::conf::get_parameter(src, &dest->node_name);
            gnd::conf::get_parameter(src, &dest->topic_name_pose);
            gnd::conf::get_parameter(src, &dest->topic_name_tracking_position);
            gnd::conf::get_parameter(src, &dest->service_name_find_objectpoint);
            gnd::conf::get_parameter(src, &dest->service_name_is_in_travelable_area);
            gnd::conf::get_parameter(src, &dest->path_map_file);
            gnd::conf::get_parameter(src, &dest->period_cui_status_display);
            gnd::conf::get_parameter(src, &dest->is_tracking_position_global);
            gnd::conf::get_parameter(src, &dest->max_distance);
            gnd::conf::get_parameter(src, &dest->angle_threshold);

            return 0;
        }

        inline
        int fwrite_node_config(const char* fname, node_config *src) {
            gnd_assert(!fname, -1, "invalid null pointer argument\n");
            gnd_assert(!src, -1, "invalid null pointer argument\n");
            {
                int ret;
                gnd::conf::file_stream fs;
                if ((ret = set_node_config(&fs, src)) < 0) return ret;
                return fs.write(fname);
            }
        }

        inline
        int set_node_config(gnd::conf::configuration *dest, node_config *src) {
            gnd_assert(!dest, -1, "invalid null pointer argument\n");
            gnd_assert(!src, -1, "invalid null pointer argument\n");

            gnd::conf::set_parameter(dest, &src->node_name);
            gnd::conf::set_parameter(dest, &src->topic_name_pose);
            gnd::conf::set_parameter(dest, &src->topic_name_tracking_position);
            gnd::conf::set_parameter(dest, &src->service_name_find_objectpoint);
            gnd::conf::set_parameter(dest, &src->service_name_is_in_travelable_area);
            gnd::conf::set_parameter(dest, &src->path_map_file);
            gnd::conf::set_parameter(dest, &src->period_cui_status_display);
            gnd::conf::set_parameter(dest, &src->is_tracking_position_global);
            gnd::conf::set_parameter(dest, &src->max_distance);
            gnd::conf::set_parameter(dest, &src->angle_threshold);

            return 0;
        }
    }
}

#endif /* HDK_FIND_OBJECTPOINT_BY_ORIENTATION_CONFIG_HPP_ */