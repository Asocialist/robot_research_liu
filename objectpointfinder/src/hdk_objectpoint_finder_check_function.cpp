/*
 * @file hdk_objectpoint_finder_check_function.cpp
 * @brief プロジェクトライブラリ内の関数を実行するプログラム．
 *
 * 引数はコマンドラインで与える．
 *
 * @author Hidekazu TAKAHASHI
 * @date 2017/11/07
 */

#include <cstdio>
#include <cstdlib>

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#include <cmath>
#undef _USE_MATH_DEFINES
#endif


#include "ros/ros.h"

#include "gnd/gnd-vector-base.hpp"
#include "gnd/gnd-matrix-base.hpp"
#include "gnd/gnd-matrix-coordinate.hpp"

#include "gnd/gnd-path.hpp"
#include "gnd/gnd-path-io.hpp"
#include "gnd_msgs/msg_path_area_and_speed_limited.h"

#include "hdk/hdk_objectpoint_finder.hpp"


typedef gnd_msgs::msg_path_area_and_speed_limited path_t;
typedef gnd::path::path_net_area_and_speed_limited path_net_t;

const char *path_file = "\\home\\kobayashilab\\ros\\locations\\bldg_RandP_5F\\bldg_RandP_5F.path";


int main(int argc, char *argv[]) {
	if ( argc != 3 ) {
		std::fprintf(stderr, "Usage: $program x y\n");
		std::exit(EXIT_FAILURE);
	}


	path_t path;
	path_net_t path_net;

	/* read path file */
	if ( gnd::path::fread( path_file, &path_net ) < 0 ) {
		std::fprintf(stderr, "[Error] Cannot read specified path file \"%s\"\n", path_file);
		return -1;
	}

	{
		double x = std::stod(argv[1], NULL);
		double y = std::stod(argv[2], NULL);
		bool ret;

		ret = hdk::objectpoint_finder::is_in_path_net(path_net, x, y, 0.0);

		std::printf("( %.2lf, %.2lf ) is %s\n", x, y, ret? "inside": "outside" );
	}

	return 0;
}
