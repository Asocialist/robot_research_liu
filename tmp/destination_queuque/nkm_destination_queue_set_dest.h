#pragma once

#include "gnd/gnd-path.hpp"
#include "gnd_msgs/msg_path_area_and_speed_limited.h"
#include "jwvehicle_simple_navigation/srv_set_navigation_path.h"
#include "hdk_pose_resetter/srv_reset_pose.h"
#include "hdk_pose_resetter/srv_get_waypoint.h"
#include "hdk_waypoint_finder/srv_is_in_travelable_area.h"
#include "hdk_waypoint_finder/srv_find_waypoint.h"

int main2(int argc, char **argv);



class NKMSetDest{
public:

    int defAttemptLimits = 10;
    std::string defServiceName_set_navigation_path      = "set_navigation_path";


private:
    gnd_msgs::msg_path_area_and_speed_limited               path;
    gnd::path::path_net_area_and_speed_limited              path_net;

    ros::ServiceClient                                      srv_client_set_navigation_path;
    jwvehicle_simple_navigation::srv_set_navigation_path    srv_set_navigation_path;


public:
    int Initialize(const char* path_file);
    int SetDestination(const char* nameDestination);
    int GetWaypointPosition(const char *name, double *x, double *y);
    bool IsCorrectWaypoint(const char *name);
    int DeleteDestination();
    gnd_msgs::msg_path_area_and_speed_limited SearchRoute(const char* nameDestination, double vehicle_position_x, double vehicle_position_y);

    void SetCurrentVehiclePosition(double x, double y);

private:
    double vehicle_pos_x;
    double vehicle_pos_y;

};


