#include <sstream>
#include <vector>
#include <algorithm>

#include "ros/ros.h"
#include "json11.hpp"

#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"

#include "gnd_msgs/msg_pose2d_stamped.h"
#include "gnd_msgs/msg_vehicle_status.h"

#include "nkm_destination_queue/AddDestination.h"
#include "nkm_destination_queue/DeleteDestination.h"
#include "nkm_destination_queue/ShowQueue.h"

#include "nkm_destination_queue_set_dest.h"

// ////////////////
// // Defines
// ////////////////


// 目的地命令データ
struct DestinationInfo{
    int index;
    std::string destName;
};

// 関数宣言
bool SrvAddDest(nkm_destination_queue::AddDestination::Request &req, nkm_destination_queue::AddDestination::Response &res);
bool SrvDelDest(nkm_destination_queue::DeleteDestination::Request &req, nkm_destination_queue::DeleteDestination::Response &res);
bool SrvShowQueue(nkm_destination_queue::ShowQueue::Request &req, nkm_destination_queue::ShowQueue::Response &res);

double DistanceP2P(double x1, double y1, double x2, double y2){
    return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}
// 状態
enum ActStatus{
    ActStatus_Idle   = 0,
    ActStatus_Wait   = 1,
    ActStatus_Move   = 2,
    ActStatus_Arrive = 3,
};

    
// ////////////////
// // Variables
// ////////////////
double posX = 0.0;
double posY = 0.0;
int latestIndex = 0;
int currentVehicleStatus = 0;
ActStatus currentActStatus = ActStatus_Idle;

std::vector<DestinationInfo> destinationQueue;
DestinationInfo currentDestination;
NKMSetDest destinationSetter;

double posDestX = 0.0;
double posDestY = 0.0;
double distnaceArriveMax = 0;

// ////////////////
// // Functions
// ////////////////

int main(int argc, char **argv){
    ros::init(argc, argv, "nkm_destination_queue");
    ros::NodeHandle nodeHandlex;

    // ---Configure---
    std::string defNodeName                 = "nkm_destination_queue";

    std::string defSrvNameAddDestination    = "nkm_destination_queue/add_destination";
    std::string defSrvNameDeleteDestination = "nkm_destination_queue/delete_destination";
    std::string defSrvNameShowQueue         = "nkm_destination_queue/show_queue";

    std::string defTopicNamePose            = "pose_particle_localizer";
    std::string defTopicNameStatus          = "vehicle_status";
    std::string defTopicNameQueueinfo       = "nkm_destination_queue/info";
    std::string defTopicNamePubDestination  = "nkm_destination_queue/next_dest";

    std::string defPathFile                 = "ros/locations/nkm_home/nkm_home.path";
    
    double defArriveDecisionDistance = 1.0;

    // ---Configure---

    {// ---Load configure---
        if (argc >= 2) {
            auto configure = json11::LoadJsonFile(argv[1]);
            if (configure == nullptr) {
                std::cerr << "cannnoot read configure file." << std::endl;
            }
            else{
                defNodeName                     = configure["defNodeName"].is_null() ? defNodeName : configure["defNodeName"].string_value();
                defSrvNameAddDestination        = configure["defSrvNameAddDestination"].is_null() ? defSrvNameAddDestination : configure["defSrvNameAddDestination"].string_value();
                defSrvNameDeleteDestination     = configure["defSrvNameDeleteDestination"].is_null() ? defSrvNameDeleteDestination : configure["defSrvNameDeleteDestination"].string_value();
                defSrvNameShowQueue             = configure["defSrvNameShowQueue"].is_null() ? defSrvNameShowQueue : configure["defSrvNameShowQueue"].string_value();
                defTopicNamePose                = configure["defTopicNamePose"].is_null() ? defTopicNamePose : configure["defTopicNamePose"].string_value();
                defTopicNameStatus              = configure["defTopicNameStatus"].is_null() ? defTopicNameStatus : configure["defTopicNameStatus"].string_value();
                defPathFile                     = configure["defPathFile"].is_null() ? defPathFile : configure["defPathFile"].string_value();
                defTopicNameQueueinfo           = configure["defTopicNameQueueinfo"].is_null() ? defTopicNameQueueinfo : configure["defTopicNameQueueinfo"].string_value();
                defTopicNamePubDestination      = configure["defTopicNamePubDestination"].is_null() ? defTopicNameQueueinfo : configure["defTopicNamePubDestination"].string_value();
                defArriveDecisionDistance       = configure["defArriveDecisionDistance"].is_null() ? defArriveDecisionDistance : configure["defArriveDecisionDistance"].int_value();
                
                destinationSetter.defAttemptLimits                      = configure["defAttemptLimits"].is_null() ? destinationSetter.defAttemptLimits : configure["defAttemptLimits"].int_value();
                destinationSetter.defServiceName_set_navigation_path    = configure["defServiceNameSetNavigationPath"].is_null() ? destinationSetter.defServiceName_set_navigation_path : configure["defServiceNameSetNavigationPath"].string_value();
                // destinationSetter.defServiceName_reset_pose             = configure["defServiceNameResetPose"].is_null() ? destinationSetter.defServiceName_reset_pose : configure["defServiceNameResetPose"].string_value();
                // destinationSetter.defServiceName_get_waypoint           = configure["defServiceNameGetWaypoint"].is_null() ? destinationSetter.defServiceName_get_waypoint : configure["defServiceNameGetWaypoint"].string_value();
                // destinationSetter.defServiceName_is_in_travelable_area  = configure["defServiceNameIsInTravelableArea"].is_null() ? destinationSetter.defServiceName_is_in_travelable_area : configure["defServiceNameIsInTravelableArea"].string_value();
                // destinationSetter.defServiceName_find_waypoint          = configure["defServiceNameFindWaypoint"].is_null() ? destinationSetter.defServiceName_find_waypoint : configure["defServiceNameFindWaypoint"].string_value();
            }

        }
        else {
            //     std::cerr << "invalid argument." << std::endl;
            //     exit(1);
        }
    }// ---Load configure---

    // ---ROS Initialize---
    ros::init(argc, argv, defNodeName.c_str());
    ros::NodeHandle nodeHandle;

    ros::ServiceServer srvRegistNode = nodeHandle.advertiseService(defSrvNameAddDestination.c_str(), SrvAddDest);
    ROS_INFO("Service Start [%s]", defSrvNameAddDestination.c_str());

    ros::ServiceServer srvUnregistNode = nodeHandle.advertiseService(defSrvNameDeleteDestination.c_str(), SrvDelDest);
    ROS_INFO("Service Start [%s]", defSrvNameDeleteDestination.c_str());

    ros::ServiceServer srvChangePriority = nodeHandle.advertiseService(defSrvNameShowQueue.c_str(), SrvShowQueue);
    ROS_INFO("Service Start [%s]", defSrvNameShowQueue.c_str());

    // ros::Publisher    publisherBridgeInfo;
    // nkm_spur_bridge::SpurBridgeInfo msgBridgeInfo;
    // publisherBridgeInfo = nodeHandle.advertise<nkm_spur_bridge::SpurBridgeInfo>(def_topicname_spurinfo.c_str(), 10);

    std_msgs::String msgNextDest;
    ros::Publisher pubNextDest = nodeHandle.advertise<std_msgs::String>(defTopicNamePubDestination.c_str(), 10);

    ros::Subscriber subPose =  nodeHandle.subscribe(defTopicNamePose.c_str(), 100, 
    +[](const gnd_msgs::msg_pose2d_stamped::ConstPtr& msg) {     
        posX = msg->x;
        posY = msg->y;
        destinationSetter.SetCurrentVehiclePosition(posX, posY);
        return;
    });

    ros::Subscriber subStatue =  nodeHandle.subscribe(defTopicNameStatus.c_str(), 100, 
    +[](const gnd_msgs::msg_vehicle_status::ConstPtr& msg) {     
        if( msg->status == gnd_msgs::msg_vehicle_status::VEHICLE_STATE_IDLE){
            if(currentActStatus == ActStatus_Move){
                //Arrive anywhere
                if(currentVehicleStatus != msg->status){   
                    currentActStatus = ActStatus_Arrive;
                }
                //Arrived already?
                else if(DistanceP2P(posX, posY, posDestX, posDestY) < distnaceArriveMax){
                    // Arrive at correct waypoint
                    currentActStatus = ActStatus_Arrive;
                }
            }
        }
        currentVehicleStatus = msg->status;
        return;
    });

    destinationSetter.Initialize(defPathFile.c_str());
    distnaceArriveMax = distnaceArriveMax;

    
    // ---Main Loop---
    ros::Rate loop_rate(100);
    while(ros::ok()){
        { // --- Publish Message ---
            //publisherBridgeInfo.publish(msgBridgeInfo);
            
        }
        switch(currentActStatus){
            case ActStatus_Idle:
                if(destinationQueue.size() != 0){
                    int st = destinationSetter.SetDestination(destinationQueue.front().destName.c_str());
                    if(st == 0){
                        ROS_INFO("Set Destination: [%s]", destinationQueue.front().destName.c_str());
                        currentDestination = destinationQueue.front();
                        destinationSetter.GetWaypointPosition(currentDestination.destName.c_str(), &posDestX, &posDestY);
                        currentActStatus = ActStatus_Move;

                        msgNextDest.data = destinationQueue.front().destName;
                        pubNextDest.publish(msgNextDest);
                    }
                    else{
                        ROS_ERROR("Destination name is wrong.(set)(name:%s)", destinationQueue.front().destName.c_str());
                    }
                }
                break;
            case ActStatus_Wait:
                break;
            case ActStatus_Move:
                break;
            case ActStatus_Arrive:
                double x, y;
                if( destinationSetter.GetWaypointPosition(currentDestination.destName.c_str(), &x, &y) >= 0){
                    if(DistanceP2P(posX, posY, x, y) < defArriveDecisionDistance){
                        // Arrive at correct waypoint
                        auto res = std::find_if( destinationQueue.begin(), destinationQueue.end(), [](DestinationInfo &i){ return(i.index == currentDestination.index);});
                        if(res != destinationQueue.end()){
                            destinationQueue.erase(res);
                        }
                    }
                    else{
                        // Arrive at wrong waypoint

                    }
                    currentActStatus = ActStatus_Idle;
                }
                else{
                    ROS_ERROR("Destination name is wrong.(arrive)(name:%s)", destinationQueue.front().destName.c_str());
                }
                break;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    // ---Main Loop---

    return 0;
}


// 目的地追加サービス
bool SrvAddDest(
        nkm_destination_queue::AddDestination::Request &req, 
        nkm_destination_queue::AddDestination::Response &res){

    if(destinationSetter.IsCorrectWaypoint(req.destination.c_str())){
        switch(req.indexInQueue){
            case nkm_destination_queue::AddDestination::Request::First:
                destinationQueue.insert(destinationQueue.begin() + 1, DestinationInfo());
                (destinationQueue.begin() + 1)->destName = req.destination.c_str();
                (destinationQueue.begin() + 1)->index = ++latestIndex;
                break;
            case nkm_destination_queue::AddDestination::Request::Current:
                destinationQueue.insert(destinationQueue.begin(), DestinationInfo());
                destinationQueue.front().destName = req.destination.c_str();
                destinationQueue.front().index = ++latestIndex;
                currentActStatus = ActStatus_Idle;
                break;
            case nkm_destination_queue::AddDestination::Request::Last:
            //case nkm_destination_queue::AddDestination::Request::Standard:
                destinationQueue.emplace_back();
                destinationQueue.back().destName = req.destination.c_str();
                destinationQueue.back().index = ++latestIndex;
                break;
            default: //index位置に挿入する仕様っぽく書いてるけどしてないよ
                destinationQueue.emplace_back();
                destinationQueue.back().destName = req.destination.c_str();
                destinationQueue.back().index = ++latestIndex;
                break;
        }


        ROS_INFO("Add Destination: [%s]", req.destination.c_str());
        res.orderID = latestIndex;
    }
    else{
        ROS_ERROR("Destination name is wrong.(srvice)(name:%s)", req.destination.c_str());
        return false;
    }

    return true;
}

// 目的地削除サービス
bool SrvDelDest(
        nkm_destination_queue::DeleteDestination::Request &req, 
        nkm_destination_queue::DeleteDestination::Response &res){

    switch(req.orderID){
        case nkm_destination_queue::DeleteDestination::Request::First:
            if(destinationQueue.size() >= 2){
                ROS_INFO("Delete Destination: [%d:%s]", (destinationQueue.begin() + 1)->index, (destinationQueue.begin() + 1)->destName.c_str());
                destinationQueue.erase(destinationQueue.begin());
            }
            else{
                ROS_INFO("Cannot Delete Destination.");
                return false;
            }
            break;
        case nkm_destination_queue::DeleteDestination::Request::Last:
            if(destinationQueue.size() >= 2){
                ROS_INFO("Delete Destination: [%d:%s]", destinationQueue.back().index, destinationQueue.back().destName.c_str());
                destinationQueue.erase(destinationQueue.end());
            }
            else{
                ROS_INFO("Cannot Delete Destination.");
                return false;
            }
            break;
        case nkm_destination_queue::DeleteDestination::Request::AllQueue:{
            ROS_INFO("Delete All Queued Destination");
            if(destinationQueue.size() >= 1){
                DestinationInfo inf = destinationQueue.front();
                destinationQueue.clear();
                destinationQueue.push_back(inf);
            }
            break;
        }    
        case nkm_destination_queue::DeleteDestination::Request::Current:
            if(destinationQueue.size() >= 1){
                ROS_INFO("Delete Destination: [%d:%s]", destinationQueue.front().index, destinationQueue.front().destName.c_str());
                destinationQueue.erase(destinationQueue.begin());
                destinationSetter.DeleteDestination();
                currentActStatus = ActStatus_Idle;
            }
            else{
                ROS_INFO("Cannot Delete Destination.");
                return false;
            }
            break;
        case nkm_destination_queue::DeleteDestination::Request::All:
            ROS_INFO("Delete All Destination");
            destinationQueue.clear();
            destinationSetter.DeleteDestination();
            currentActStatus = ActStatus_Idle;
            break;
        default:
            auto r = std::find_if( destinationQueue.begin(), destinationQueue.end(), [req](DestinationInfo &i){ return(i.index == req.orderID);});
            if(r != destinationQueue.end()){
                ROS_INFO("Delete Destination: [%d:%s]", r->index, r->destName.c_str());
                if(r == destinationQueue.begin()){
                    destinationSetter.DeleteDestination();
                    currentActStatus = ActStatus_Idle;
                }
                destinationQueue.erase(r);
            }
            else{
                ROS_ERROR("Could not find index: [%d]", req.orderID);
                return false;
            }
            break;
    }

    return true;
}

// 目的地一覧
bool SrvShowQueue(
        nkm_destination_queue::ShowQueue::Request &req, 
        nkm_destination_queue::ShowQueue::Response &res){

    ROS_INFO("Queue:");
    res.num = destinationQueue.size();
    for(const auto& itr : destinationQueue){
        ROS_INFO("    %d:%s",itr.index, itr.destName.c_str());
        res.destination.emplace_back(itr.destName.c_str());
        res.orderID.emplace_back(itr.index);
    }

    return true;
}
