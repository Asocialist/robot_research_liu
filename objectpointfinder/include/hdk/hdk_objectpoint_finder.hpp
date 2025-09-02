/*
 * @file hdk_objectpoint_finder.hpp
 * @author Hidekazu TAKAHASHI
 * @date 2017/11/14
 */

#ifndef HDK_OBJECTPOINT_FINDER_HPP_
#define HDK_OBJECTPOINT_FINDER_HPP_

#include <cstddef>
#include <cmath>

#include "hdk_objectpoint_finder/srv_find_objectpoint.h"
#include "hdk_objectpoint_finder/srv_is_in_travelable_area.h"

#include "gnd/gnd-vector-base.hpp"
#include "gnd/gnd-matrix-base.hpp"
#include "gnd/gnd-matrix-coordinate.hpp"

#include "hdk/hdk_objectpoint_finder_config.hpp"



namespace hdk {
	namespace objectpoint_finder {

		typedef struct {
			double x;
			double y;
		} point2d_t;

		typedef struct {
			double x;
			double y;
			double theta;
		} pose2d_t;


		/**
		 * @brief 地図座標系の点を指定した経路座標系に変換する関数．
		 *        始点と終点で経路を指定するが，存在しないときは失敗し，dstは不定．
		 * @param[in] path_net パスネット
		 * @param[in] objectpoint_name_start 経路の始点
		 * @param[in] objectpoint_name_end 経路の終点
		 * @param[in] src 点（地図座標系）
		 * @param[out] dst 点（経路座標系）
		 * @return true 成功, false 失敗
		 */
		bool coordtf_map_to_path( gnd::path::path_net_area_and_speed_limited &path_net,
				const char *objectpoint_name_start, const char *objectpoint_name_end,
				const point2d_t &src, point2d_t &dst);


		/**
		 * @brief 指定した点が経路の中にあるか判定する関数．
		 * @param[in] path_net パスネット
		 * @param[in] objectpoint_name_start 経路の始点
		 * @param[in] objectpoint_name_end 経路の終点
		 * @param[in] x 位置（地図座標系）
		 * @param[in] y 位置（地図座標系）
		 * @param[in] expansion Configuration-spaceで膨張させる量
		 *                      ロボットの大きさ（ロボット座標系中心から最も離れている部分までの長さ）を設定するとよい
		 * @return true 存在する, flase それ以外
		 * @note パス自体が存在しない場合はfalseを返す．
		 */
		bool is_in_path( gnd::path::path_net_area_and_speed_limited &path_net,
				const char *objectpoint_name_start, const char *objectpoint_name_end,
				double x, double y, double expansion );


		/**
		 * @brief 指定した点が経路の中にあるか判定する関数．
		 * @param[in] path_net パスネット
		 * @param[in] x 位置（地図座標系）
		 * @param[in] y 位置（地図座標系）
		 * @param[in] expansion Configuration-spaceで膨張させる量
		 *                      ロボットの大きさ（ロボット座標系中心から最も離れている部分までの長さ）を設定するとよい
		 * @return true 存在する, flase それ以外
		 */
		bool is_in_path_net( gnd::path::path_net_area_and_speed_limited &path_net, double x, double y, double expansion );


		/**
		 * @brief 指定した位置に最も近い経路点(objectpoint)を返す関数．
		 *        指定した位置が走行可能領域でないときは，失敗し，objectpoint_nameは不定．
		 * @param[in] path_net パスネット
		 * @param[in] x 位置（地図座標系）
		 * @param[in] y 位置（地図座標系）
		 * @param[in] expansion Configuration-spaceで膨張させる量
		 *                      ロボットの大きさ（ロボット座標系中心から最も離れている部分までの長さ）を設定するとよい
		 * @param[in] objectpoint_name_destination 目的地の名前
		 * @param[out] objectpoint_name 最も近い経路点の名前
		 * @return true 成功, false 失敗
		 */
		bool find_objectpoint( gnd::path::path_net_area_and_speed_limited &path_net,
				double x, double y, double expansion,
				const char *objectpoint_name_destination, char *objectpoint_name );


		/**
		 * @brief パスの長さ（道のり）を返す関数．
		 * @param[in] path
		 * @return パスの長さ
		 */
		double get_path_length(gnd::path::path_net_area_and_speed_limited::path_t &path);



		// ---> callback function object
		class srv_funcobj_find_objectpoint {
		public:
			typedef hdk_objectpoint_finder::srv_find_objectpointRequest request_t;
			typedef hdk_objectpoint_finder::srv_find_objectpointResponse response_t;

		public:
			srv_funcobj_find_objectpoint(gnd::path::path_net_area_and_speed_limited &path_net);
			~srv_funcobj_find_objectpoint();

		public:
			void set_pose(pose2d_t &pose);
			std::size_t get_count_callback();

		public:
			bool callback(request_t &request, response_t &response);

		private:
			gnd::path::path_net_area_and_speed_limited path_net_;
			pose2d_t pose_;
			std::size_t count_callback_;
		};


		class srv_funcobj_is_in_travelable_area {
		public:
			typedef hdk_objectpoint_finder::srv_is_in_travelable_areaRequest request_t;
			typedef hdk_objectpoint_finder::srv_is_in_travelable_areaResponse response_t;

		public:
			srv_funcobj_is_in_travelable_area(gnd::path::path_net_area_and_speed_limited &path_net);
			~srv_funcobj_is_in_travelable_area();

		public:
			void set_pose(pose2d_t &pose);

		public:
			bool callback(request_t &request, response_t &response);

		private:
			gnd::path::path_net_area_and_speed_limited path_net_;
			pose2d_t pose_;

		};
		// <--- callback function object

	}
}



namespace hdk {
	namespace objectpoint_finder {


		inline
		bool coordtf_map_to_path( gnd::path::path_net_area_and_speed_limited &path_net,
				const char *objectpoint_name_start, const char *objectpoint_name_end,
				const point2d_t &src, point2d_t &dst) {
			gnd::path::path_net_area_and_speed_limited::property_t prop;
			gnd::vector::fixed_column<4> point_on_map;
			gnd::vector::fixed_column<4> point_on_path;

			/* check if specified path exists */
			if (path_net.get_path_property( objectpoint_name_start, objectpoint_name_end, &prop ) < 0 ) {
				/* there is not specified path */
				return false;
			}

			{ // ---> coordinate transiform
				gnd::matrix::fixed<4,4> mat_coordtf_map_to_path; // 変換行列
				point2d_t objectpoint_start;         // 経路点（地図座標系）
				point2d_t objectpoint_end;           // 経路点（地図座標系）
				point2d_t objectpoint_start_on_path; // 経路点（経路座標系）
				point2d_t offset;         // 経路座標系原点から見た地図座標系原点の位置
				double theta;            // 経路座標系の角度（地図座標系）

				path_net.get_objectpoint( objectpoint_name_start, &objectpoint_start.x, &objectpoint_start.y );
				path_net.get_objectpoint( objectpoint_name_end, &objectpoint_end.x, &objectpoint_end.y );
				theta = std::atan2(objectpoint_end.y - objectpoint_start.y, objectpoint_end.x - objectpoint_start.x);

				{ // ---> find offset
					offset.x = -( objectpoint_end.x * std::cos(theta) + objectpoint_end.y * std::sin(theta) );
					offset.y = -( -objectpoint_end.x * std::sin(theta) + objectpoint_end.y * std::cos(theta) );
				} // <--- find offset

				gnd::matrix::coordinate_converter( &mat_coordtf_map_to_path,
						offset.x, offset.y, 0.0,
						std::cos(-theta), std::sin(-theta), 0.0,
						0.0, 0.0, 1.0);

				{ // ---> transform
					/* set source */
					point_on_map[0] = src.x;
					point_on_map[1] = src.y;
					point_on_map[2] = 0.0;
					point_on_map[3] = 1.0;
					/* transform  */
					gnd::matrix::prod( &mat_coordtf_map_to_path, &point_on_map, &point_on_path );
					/* set return value */
					dst.x = point_on_path[0];
					dst.y = point_on_path[1];
				} // <--- transform

				/*std::printf("theta: %.2lf\n", theta);
				std::printf("offset: ( %.2lf %.2lf )\n", offset.x, offset.y);
				std::printf("[\"%s\" -> \"%s\"]: ( %.2lf, %.2lf ) -> ( %.2lf, %.2lf )\n\n",
						objectpoint_name_start, objectpoint_name_end,
						src.x, src.y, dst.x, dst.y);*/
			}
			return true;
		}


		inline
		bool is_in_path( gnd::path::path_net_area_and_speed_limited &path_net,
				const char *objectpoint_name_start, const char *objectpoint_name_end,
				double x, double y, double expansion ) {
			gnd::path::path_net_area_and_speed_limited::property_t prop;
			gnd::vector::fixed_column<4> point_on_map;
			gnd::vector::fixed_column<4> point_on_path;

			/* check specified path exist */
			if (path_net.get_path_property( objectpoint_name_start, objectpoint_name_end, &prop ) < 0 ) {
				/* there is not specified path */
				return false;
			}


			{ // ---> coordinate transiform
				gnd::matrix::fixed<4,4> mat_coordtf_map_to_path; // 変換行列
				point2d_t objectpoint_start;         // 経路点（地図座標系）
				point2d_t objectpoint_start_on_path; // 経路点（経路座標系）
				point2d_t target;                 // 調べたい点（地図座標系）
				point2d_t target_on_path;         // 調べたい点（経路座標系）

				path_net.get_objectpoint( objectpoint_name_start, &objectpoint_start.x, &objectpoint_start.y );
				target.x = x;
				target.y = y;

				{ // ---> transform
					coordtf_map_to_path( path_net, objectpoint_name_start, objectpoint_name_end, objectpoint_start, objectpoint_start_on_path );
					coordtf_map_to_path( path_net, objectpoint_name_start, objectpoint_name_end, target, target_on_path );
				} // <--- transform

				{ // ---> evaluate
					if ( target_on_path.x > prop.end_extend ) {
						return false;
					} else if ( target_on_path.x < objectpoint_start_on_path.x - prop.start_extend ) {
						return false;
					} else if ( target_on_path.y > prop.left_width - expansion ) {
						return false;
					} else if ( target_on_path.y < -prop.right_width + expansion ) {
						return false;
					} else {
						return true;
					}
				} // <--- evaluate
			} // <--- coordinate transiform
		}


		inline
		bool is_in_path_net( gnd::path::path_net_area_and_speed_limited &path_net, double x, double y, double expansion ) {
			std::size_t i;
			std::size_t j;
			char name_i[128];
			char name_j[128];

			// ---> scanning path
			for ( i=0; i<path_net.n_objectpoints(); i++ ) {
				for ( j=i+1; j<path_net.n_objectpoints(); j++ ) {
					/* get name */
					path_net.name_objectpoint(i, name_i);
					path_net.name_objectpoint(j, name_j);
					/* check */
					/* 経路は方向別にプロパティを設定可能なため，双方向調べる */
					if ( is_in_path( path_net, name_i, name_j, x, y, expansion ) ) {
						return true;
					}
					std::printf("\"%s\" -> \"%s\": %s\n", name_i, name_j, "false");
					if ( is_in_path( path_net, name_j, name_i, x, y, expansion ) ) {
						return true;
					}
					std::printf("\"%s\" -> \"%s\": %s\n", name_j, name_i, "false");
				}
			} // <--- scanning path

			return false;
		}


	bool find_objectpoint( gnd::path::path_net_area_and_speed_limited &path_net,
        double x, double y, double expansion,
        const char *objectpoint_name_destination, char *objectpoint_name ) {
    std::size_t i;
    double min_distance = std::numeric_limits<double>::max();
    int closest_index = -1;
    char current_name[128];

    /* check destination only if it's not empty */
    if (objectpoint_name_destination[0] != '\0' && path_net.index_objectpoint(objectpoint_name_destination) == -1) {
        /* there is no specified objectpoint */
        return false;
    }

    // Find the closest objectpoint
    for ( i = 0; i < path_net.n_objectpoints(); i++ ) {
        double obj_x, obj_y;
        path_net.name_objectpoint(i, current_name);
        path_net.get_objectpoint(current_name, &obj_x, &obj_y);
        
        double distance = std::sqrt(std::pow(x - obj_x, 2) + std::pow(y - obj_y, 2));
        
        if (distance < min_distance) {
            min_distance = distance;
            closest_index = i;
        }
    }

    if (closest_index != -1) {
        path_net.name_objectpoint(closest_index, objectpoint_name);
        return true;
    }

    return false;
}


		inline
		double get_path_length(gnd::path::path_net_area_and_speed_limited::path_t &path) {
			double length = 0.0;
			if ( path.path.size() > 0 ) {
				{
					double dx;
					double dy;
					dx = path.path[0].end.x - path.start.x;
					dy = path.path[0].end.y - path.start.y;
					length += std::sqrt(dx*dx + dy*dy);
				}

				for ( std::size_t i=1; i<path.path.size(); i++ ) {
					double dx;
					double dy;
					dx = path.path[i].end.x - path.path[i-1].end.x;
					dy = path.path[i].end.y - path.path[i-1].end.y;
					length += std::sqrt(dx*dx + dy*dy);
				}
			}
			return length;
		}


		// ---> callback function object
		inline
		srv_funcobj_find_objectpoint::srv_funcobj_find_objectpoint(gnd::path::path_net_area_and_speed_limited &path_net):
			path_net_(path_net), count_callback_(0) {
			pose_.x = 0.0;
			pose_.y = 0.0;
			pose_.theta = 0.0;
		}

		inline
		srv_funcobj_find_objectpoint::~srv_funcobj_find_objectpoint() {
			/* nothing to do */
		}

		inline
		void srv_funcobj_find_objectpoint::set_pose(pose2d_t &pose) {
			pose_ = pose;
		}

		inline
		std::size_t srv_funcobj_find_objectpoint::get_count_callback() {
			return count_callback_;
		}

		inline
bool srv_funcobj_find_objectpoint::callback(request_t &request, response_t &response) {
    char objectpoint_name[128];

    /* count */
    count_callback_++;

    /* find closest objectpoint regardless of travelable area */
    bool ret_value;
    ret_value = find_objectpoint( path_net_, pose_.x, pose_.y, 0.0,
                    request.objectpoint_name_destination.c_str(), objectpoint_name );
    response.objectpoint_name = std::string(objectpoint_name);

    if (ret_value) {
        std::cout << "Closest objectpoint: " << objectpoint_name << std::endl;
    } else {
        std::cout << "No objectpoint found." << std::endl;
    }

    return ret_value;
}

		inline
		srv_funcobj_is_in_travelable_area::srv_funcobj_is_in_travelable_area(gnd::path::path_net_area_and_speed_limited &path_net):
			path_net_(path_net) {
			pose_.x = 0.0;
			pose_.y = 0.0;
			pose_.theta = 0.0;
		}

		inline
		srv_funcobj_is_in_travelable_area::~srv_funcobj_is_in_travelable_area() {
			/* nothing to do */
		}

		inline
		void srv_funcobj_is_in_travelable_area::set_pose(pose2d_t &pose) {
			pose_ = pose;
		}

		inline
		bool srv_funcobj_is_in_travelable_area::callback(request_t &request, response_t &response) {
			/* check if the robot is in movable area */
			if ( !is_in_path_net( path_net_, pose_.x, pose_.y, 0.0 ) ) {
				/* out of area */
				response.ret = false;
			} else {
				response.ret = true;
			}

			return true;
		}
		// <--- callcabk function object

	}
}


#endif /* HDK_OBJECTPOINT_FINDER_HPP_ */
