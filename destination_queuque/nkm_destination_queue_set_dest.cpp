/*****************************************************************************
** Includes
*****************************************************************************/

#include <string>
#include <vector>

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "gnd/gnd-util.h"
#include "gnd/gnd-path.hpp"
#include "gnd/gnd-path-io.hpp"
#include "../include/path_utility.hpp"
#include "nkm_destination_queue_set_dest.h"

/*****************************************************************************
** Defines
*****************************************************************************/
constexpr int DEF_MAX_LENGTH_WAYPOINT_NAME = 128;
constexpr double DEF_EXPANSION = 0;
constexpr double DEF_MAX_WEIGHT_ROUTE = DBL_MAX;
/*****************************************************************************
** Functions
*****************************************************************************/
int NKMSetDest::Initialize(const char* path_file){
    ros::NodeHandle nodeHandle;

    srv_client_set_navigation_path      = nodeHandle.serviceClient<jwvehicle_simple_navigation::srv_set_navigation_path>(defServiceName_set_navigation_path.c_str());

    /* read path file */
    if ( gnd::path::fread( path_file, &path_net ) < 0 ) {
        ROS_ERROR("Cannot read specified path file \"%s\"", path_file);
        return -1;
    }
}

typedef struct {
    int start_waypoint_index;
    gnd::path::path_unit_area_and_speed_limited *path = nullptr;
    gnd::path::path_net_area_and_speed_limited::path_t ws;
    double weight = DEF_MAX_WEIGHT_ROUTE;
    bool available = true;
    bool use_first_path = false;
} route_candidate;

int NKMSetDest::SetDestination(const char* nameDestination){
    int destination_waypoint_index = -1;

    // Get and check Destination waypoint index.
    destination_waypoint_index = path_net.index_waypoint(nameDestination);
    if ( destination_waypoint_index < 0 ) {
        ROS_ERROR("There is not specified waypoint (Dest)\"%s\"", nameDestination);
        return -1;
    }

    auto path = SearchRoute(nameDestination, vehicle_pos_x, vehicle_pos_y);

    bool ret;
    srv_set_navigation_path.request.path = path;
    ret = srv_client_set_navigation_path.call(srv_set_navigation_path);
    std::fprintf(stderr, "%s\n", ret? "true": "false"); 

    return 0;

}

gnd_msgs::msg_path_area_and_speed_limited NKMSetDest::SearchRoute(const char* nameDestination, double vehicle_position_x, double vehicle_position_y){
    int destination_waypoint_index = -1;
    std::vector<route_candidate> route_candidate_list;

    // Get and check Destination waypoint index.
    destination_waypoint_index = path_net.index_waypoint(nameDestination);
    if ( destination_waypoint_index < 0 ) {
        ROS_ERROR("There is not specified waypoint (Dest)\"%s\"", nameDestination);
        return gnd_msgs::msg_path_area_and_speed_limited();
    }

    // 自身のいるパス(片方向)をすべて取得して、ルートの候補の起点とする
    for (int i = 0; i < path_net.n_waypoints(); i++){
        for (int j = 0; j < path_net[i].list.size(); j++){
            if(path_util::is_in_path(path_net[i].waypoint, path_net[i].list[j], vehicle_position_x, vehicle_position_y, DEF_EXPANSION)){
                route_candidate candidate;
                candidate.start_waypoint_index = i;
                candidate.path = &path_net[i].list[j];
                route_candidate_list.emplace_back(candidate);
            }
        }
    }
    // 候補ごとに処理
    for (auto& candidate : route_candidate_list){    
        //gnd::path::path_net_area_and_speed_limited::path_t ws;    
        {// 自身のいるパスの終点Waypointからゴールまでの経路を探索
            int ret = path_net.find_path_dijkstra(&(candidate.ws),candidate.path->end.name, nameDestination);
            if ( ret < 0 ){
                // 経路が見つからない場合、候補を使わない
                candidate.available = false;
                continue;
            }
        }
        
        {//　経路weight計算
            candidate.weight = 0;
            gnd::path::waypoint_named_t start;
            start.x = vehicle_pos_x; start.y = vehicle_pos_y;
            strcpy(start.name,"vehicle");

            //　経路がない（終点のみの場合）
            if (candidate.ws.path.size() == 0){
                //ROS_INFO("route length is zero");
                candidate.use_first_path = true;
                candidate.weight += path_util::calculate_weight(start, candidate.path->end, candidate.path->prop);
                continue;
            }
            // 経路に自身のいるパスは含まれるか？
            if (path_util::is_in_path(candidate.path->end, candidate.ws.path[0], vehicle_pos_x, vehicle_pos_y, DEF_EXPANSION)){
                //Yes->そのまま、経路のweightを計算する
                candidate.use_first_path = false;
                candidate.weight += path_util::calculate_weight(start, candidate.ws.path[0].end, candidate.ws.path[0].prop);
                for (int i = 1; i < candidate.ws.path.size(); i++){
                    candidate.weight += path_util::calculate_weight(candidate.ws.path[i-1].end, candidate.ws.path[i].end, candidate.ws.path[i].prop);
                }
                //ROS_INFO("TRUE:%.3f", candidate.weight);
            }
            else{
                //No->現在のパスを含めて、weightの計算をする
                candidate.use_first_path = true;
                candidate.weight += path_util::calculate_weight(start, candidate.path->end, candidate.path->prop);
                candidate.weight += path_util::calculate_weight(candidate.path->end, candidate.ws.path[0].end, candidate.ws.path[0].prop);
                for (int i = 1; i < candidate.ws.path.size(); i++){
                    candidate.weight += path_util::calculate_weight(candidate.ws.path[i-1].end, candidate.ws.path[i].end, candidate.ws.path[i].prop);
                }
                //ROS_INFO("FALSE:%.3f", candidate.weight);
            }
        }
        //candidate.ws = ws;
    }
    // 利用可能かつ、最もweightの少ない経路を決定
    double min_weight = DEF_MAX_WEIGHT_ROUTE;
    double min_index = -1;
    for (int i = 0; i < route_candidate_list.size(); i++){
        if (route_candidate_list[i].available == false)
            continue;
        if (route_candidate_list[i].weight < min_weight){
            min_index = i;
            min_weight = route_candidate_list[i].weight;
            //ROS_INFO("min:%.3f,%d", min_weight, i);
        }
    }



    {// ルートのmessageを生成
        
        path.path.clear();

        // ルートがない場合、空のルートを返す
        if (min_index == -1){
            return path;
        }
        
        // 始点
        gnd::path::waypoint_named_t start_waypoint;
        if(route_candidate_list[min_index].use_first_path == true){
            // start
            start_waypoint = path_net[route_candidate_list[min_index].start_waypoint_index].waypoint;
            // first path
            path.path.emplace_back(path_util::convert_path_unit_message(route_candidate_list[min_index].path));
            ROS_INFO("USE FIRST PATH");
        }   
        else{
            // start
            start_waypoint = route_candidate_list[min_index].path->end;
             ROS_INFO("NOT USE FIRST PATH");
        }
        ROS_INFO("%s",start_waypoint.name);
        path.start.name = start_waypoint.name;
        path.start.x = start_waypoint.x;
        path.start.y = start_waypoint.y;
        path.start.theta = 0;

        for (auto p : route_candidate_list[min_index].ws.path){
            //auto p2 = ;
            path.path.emplace_back(path_util::convert_path_unit_message(&p));
        }
        return path;
    }
    //return path;
}

int NKMSetDest::GetWaypointPosition(const char *name, double *x, double *y){
    return path_net.get_waypoint(name, x, y);
}

bool NKMSetDest::IsCorrectWaypoint(const char *name){
    double x, y;
    return (path_net.get_waypoint(name, &x, &y) >= 0) ? true : false;
}

int NKMSetDest::DeleteDestination(){
    // Call Blank Path.
    if(path_net.n_waypoints() != 0){
        srv_set_navigation_path.request.path.start.name = path_net[0].waypoint.name;
        srv_set_navigation_path.request.path.start.x = path_net[0].waypoint.x;
        srv_set_navigation_path.request.path.start.y = path_net[0].waypoint.y;
    }
    srv_set_navigation_path.request.path.path.clear();
    bool ret = srv_client_set_navigation_path.call(srv_set_navigation_path);
    return 0;
}
void  NKMSetDest::SetCurrentVehiclePosition(double x, double y){
    vehicle_pos_x = x;
    vehicle_pos_y = y;
}


/*****************************************************************************
** Utils
*****************************************************************************/

int main2(int argc, char **argv){
    NKMSetDest destinationSetter;
    destinationSetter.Initialize("/home/kobayashilab/ros/locations/jw_1213-2/jw_1213-2.path");
    destinationSetter.SetCurrentVehiclePosition(3, 0);
    destinationSetter.SetDestination("003");
    return 0;
}

