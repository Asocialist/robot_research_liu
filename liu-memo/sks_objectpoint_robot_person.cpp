#include <stdio.h>
#define _USE_MATH_DEFINES
#define ROOT 0.87
#include <cmath>
#include <string>
#include "json11.hpp"
#include "DataTypedef.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose2D.h"
#include "gnd_msgs/msg_pose2d_stamped.h"
#include "gnd_msgs/msg_waypoint_named.h"

#include <ypspur.h>

#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include "gnd_msgs/msg_Fformation_mode.h"

#ifndef M_PI
#define M_PI 3.14159265
#endif


#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"

// ---Defines---    
// ---Defines---  
// ---Configure---
std::string defNodeName = "sks_objectpoint_robot_person";
std::string defTopicNameTrackingPosition = "pose_person_following";
std::string defTopicNameVehiclePose = "pose_particle_localizer";
// std::string defPathFileName = "/home/kobayashilab/ros/locations/shopping_20241211/shopping_20241211.path";
//std::string defPathFileName = "/home/kobayashilab/ros/locations/bldg_RandP_5F/bldg_RandP_5F.path";
//std::string defPathFileName = "/home/kobayashilab/ros/locations/shino_5f/shino_5f.path";
std::string defPathFileName = "/home/kobayashilab/ros/locations/250603/250603.path";

double defRediscoverySecRate = 2.0;
double defDistanceArrivePoint = 0.15;
bool   defIsRegistDestinaionAuto = true;
bool   defIsTrackingPositionGlobal = false;

bool flag = false;  // フラグをグローバル変数として宣言

// ---Configure---

// ---Variables---                          
Vec2d_t poseTracking = Vec2d_t(0.0, 0.0);
Vec2d_t poseVehicle = Vec2d_t(0.0, 0.0);
double  poseVehicleT = 0;
double  poseTrackingT = 0;

//double distance = 100;
//double posY = 0;
ros::Time timeLastRediscovery;

bool isReceivedrobotpose = false;

bool isReceivedLatestTracking = false;
bool isMovingToTracking = false;
Vec2d_t pointTrackingOnPath;
int indexDestQueueOrder = 0;
// ---Variables---  

std::string latest_objectpoint_name_;
double latest_objectpoint_x_;
double latest_objectpoint_y_;
bool has_received_data_ = false;

//sbtpへ
ros::Publisher formationModePub;

// グローバル変数として追加
struct TrackingStatus {
    ros::Time lastUpdateTime;
    bool isTracking;
    static constexpr double TIMEOUT_DURATION = 1.0;  // 追跡タイムアウトの閾値（秒）
} trackingStatus;

//void closestObjectpointCallback(const hdk_objectpoint_finder::ObjectPointInfo::ConstPtr& msg)
/*
void closestObjectpointCallback(const gnd_msgs::msg_waypoint_named::ConstPtr& msg)
{
    latest_objectpoint_name_ = msg->name;
    latest_objectpoint_x_ = msg->x;
    latest_objectpoint_y_ = msg->y;
    has_received_data_ = true;

        // デバッグ出力を追加
    ROS_INFO("Received objectpoint: %s at (%.2f, %.2f)", 
        msg->name.c_str(), msg->x, msg->y);
}
*/

void closestObjectpointCallback(const gnd_msgs::msg_waypoint_named::ConstPtr& msg) {
    if (!trackingStatus.isTracking) {
        // 追跡していない場合は、オブジェクトポイントを更新しない
        return;
    }
    latest_objectpoint_name_ = msg->name;
    latest_objectpoint_x_ = msg->x;
    latest_objectpoint_y_ = msg->y;
    has_received_data_ = true;

   ROS_WARN("Received objectpoint: %s at (%.2f, %.2f)", 
        msg->name.c_str(), msg->x, msg->y);


    // オブジェクトポイントと人との距離を計算
    double distance_to_person = sqrt(
        pow(latest_objectpoint_x_ - poseTracking.x, 2.0) + 
        pow(latest_objectpoint_y_ - poseTracking.y, 2.0)
    );
    
    // 1.3m以内にオブジェクトポイントがある場合は自動的にフラグをtrue に設定
    if(distance_to_person <= 1.3){//1.0の時精度良い
        flag = true;
        ROS_INFO(GREEN"TRUE!!!!!!!"RESET);
        ROS_INFO(BLUE"-------(%.2f)-------"RESET,distance_to_person);
    }else{
    flag = false;
    printf("もっと近づいてください");
}
    ROS_INFO("Received objectpoint: %s at (%.2f, %.2f), distance to person: %.2f, flag: %s", 
        msg->name.c_str(), msg->x, msg->y, distance_to_person, flag ? "true" : "false");
}

void personPositionCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    ros::Time currentTime = ros::Time::now();
    
    if(defIsTrackingPositionGlobal){
        poseTracking.x = msg->x;
        poseTracking.y = msg->y;
    } else {
        poseTracking.x = msg->x * cos(poseVehicleT) - msg->y * sin(poseVehicleT) + poseVehicle.x;
        poseTracking.y = msg->x * sin(poseVehicleT) + msg->y * cos(poseVehicleT) + poseVehicle.y;
        poseTrackingT = msg->theta + poseVehicleT;
    }
    
    trackingStatus.lastUpdateTime = currentTime;
    trackingStatus.isTracking = true;
    isReceivedLatestTracking = true;
}

int main(int argc, char **argv) {

    {//---Load configure---
        if (argc >= 2) {
            auto configure = json11::LoadJsonFile(argv[1]);
            if (configure == nullptr) {
                std::cerr << "Cannot read configure file." << std::endl;
                exit(1);
            }

            defNodeName                             = configure["defNodeName"].is_null() ? defNodeName : configure["defNodeName"].string_value();
            defTopicNameTrackingPosition            = configure["defTopicNameTrackingPosition"].is_null() ? defTopicNameTrackingPosition : configure["defTopicNameTrackingPosition"].string_value();
            defTopicNameVehiclePose                 = configure["defTopicNameVehiclePose"].is_null() ? defTopicNameVehiclePose : configure["defTopicNameVehiclePose"].string_value();
            defRediscoverySecRate                   = configure["defRediscoverySecRate"].is_null() ? defRediscoverySecRate : configure["defRediscoverySecRate"].number_value();
            defDistanceArrivePoint                  = configure["defDistanceArrivePoint"].is_null() ? defDistanceArrivePoint : configure["defDistanceArrivePoint"].number_value();
            defIsRegistDestinaionAuto               = configure["defIsRegistDestinaionAuto"].is_null() ? defIsRegistDestinaionAuto : configure["defIsRegistDestinaionAuto"].bool_value();
            defIsTrackingPositionGlobal             = configure["defIsTrackingPositionGlobal"].is_null() ? defIsRegistDestinaionAuto : configure["defIsTrackingPositionGlobal"].bool_value();
        }
    }//---Load configure---

    // ---ROS Initialize---
    ros::init(argc, argv, defNodeName.c_str());
    ros::NodeHandle nodeHandle;

    // Publish -> sbtpへ
    formationModePub = nodeHandle.advertise<gnd_msgs::msg_Fformation_mode>("formation_topic", 10);


    // ロボットの位置情報サブスクライブ
    ros::Subscriber subVehiclePosition = nodeHandle.subscribe(defTopicNameVehiclePose.c_str(), 100, 
    +[](const gnd_msgs::msg_pose2d_stamped::ConstPtr& msg) {     
        poseVehicle.x = msg->x;
        poseVehicle.y = msg->y; 
        poseVehicleT  = msg->theta;     
        isReceivedrobotpose = true;
        return;
    }); 
      // メイン関数内でのサブスクライバーの設定
    ros::Subscriber subPersonPosition = nodeHandle.subscribe(
        defTopicNameTrackingPosition.c_str(), 
        100, 
        personPositionCallback
    );

    // ロボットに最も近いclosest_objectpointをサブスクライブ
    ros::Subscriber sub = nodeHandle.subscribe("closest_objectpoint", 10, closestObjectpointCallback);

    //---Main Loop---
    timeLastRediscovery = ros::Time::now();
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        ros::spinOnce();

        // 追跡状態の確認と更新
        ros::Time currentTime = ros::Time::now();
         
        if ((currentTime - trackingStatus.lastUpdateTime).toSec() > TrackingStatus::TIMEOUT_DURATION) {
            if (trackingStatus.isTracking) {
                ROS_WARN("Lost person tracking");
                trackingStatus.isTracking = false;
                has_received_data_ = false;  // オブジェクトポイントデータを無効化
                flag = false;  // 追跡を失った場合はフラグをfalseに設定
            }
        }

       // if (has_received_data_ && isReceivedLatestTracking && isReceivedrobotpose) {
       // if (isReceivedLatestTracking && isReceivedrobotpose) {
      if (trackingStatus.isTracking && isReceivedrobotpose) {
            // sbtpへ
            gnd_msgs::msg_Fformation_mode msg;

        // 人とオブジェクトポイント間の距離を計算
            double distance_opt = sqrt(pow(latest_objectpoint_x_ - poseTracking.x, 2.0) + pow(latest_objectpoint_y_ - poseTracking.y, 2.0));

          //  if (has_received_data_) {  // has_received_data_のチェックを追加
            if (flag && has_received_data_ && distance_opt <= 1.3) {  // has_received_data_のチェックを追加

            ROS_INFO("Fformation OK!!!");


            

            if (distance_opt >= 0.01 && distance_opt < 0.5) {
                distance_opt = 0.50;
            } 
            else if (distance_opt >= 0.5 && distance_opt < 0.6) {
                distance_opt = 0.60;
            }
            else if (distance_opt >= 0.6 && distance_opt < 0.7) {
                distance_opt = 0.70;
            }
            else if (distance_opt >= 0.7 && distance_opt < 0.8) {
                distance_opt = 0.8;
            }
            else if (distance_opt >= 0.8 && distance_opt < 0.9) {
                distance_opt = 0.9;
            }
            else if (distance_opt >= 0.9 && distance_opt < 1.0) {
                distance_opt = 1.0;
            }
            else if (distance_opt >= 1.0 && distance_opt < 1.1) {
                distance_opt = 1.10;
            }
            else if (distance_opt >= 1.1 && distance_opt < 1.2) {
                distance_opt = 1.20;
            }
            else if (distance_opt >= 1.2 && distance_opt < 1.3) {
                distance_opt = 1.30;
            }
            else if (distance_opt >= 1.3 && distance_opt < 1.4) {
                distance_opt = 1.40;
            }
            else if (distance_opt >= 1.4 && distance_opt < 1.5) {
                distance_opt = 1.50;
            }
            else if (distance_opt >= 1.5 && distance_opt < 1.6) {
                distance_opt = 1.60;
            }
            else if (distance_opt >= 1.6 && distance_opt < 1.7) {
                distance_opt = 1.70;
            }
            else if (distance_opt >= 1.7 && distance_opt < 1.8) {
                distance_opt = 1.80;
            }

 // オブジェクトポイントが1.3m以内にある場合のみフォーメーションモードを有効化
          //  if (flag) {
            // objectpointと人物の中点を計算
            // objectpointと人物の中点を計算
            double midX = (latest_objectpoint_x_ + poseTracking.x) / 2.0;
            double midY = (latest_objectpoint_y_ + poseTracking.y) / 2.0;

            // ロボットから中点までの距離と角度を計算
            double diffX = midX - poseVehicle.x;
            double diffY = midY - poseVehicle.y;
            double distance2 = sqrt(pow(diffX, 2.0) + pow(diffY, 2.0));
            double Theta2 = atan2(diffY, diffX) - poseVehicleT;

            while (Theta2 > M_PI) Theta2 -= 2 * M_PI;
            while (Theta2 < -M_PI) Theta2 += 2 * M_PI;

            // 中間点からの最小距離を0.45mに設定
            double min_distance = 0.7;
            // デッドゾーンの定義
            const double DEAD_ZONE = 0.3;  // 5cm程度の許容範囲

            // デッドゾーンロジックの追加
            if (std::abs(distance2 - min_distance) < DEAD_ZONE) {
                // デッドゾーン内であれば完全に停止
                msg.linear_velocity = 0.0;
            } 
            else if (distance2 < min_distance) {
                // ロボットが最小距離より近い場合、後退させる
                msg.linear_velocity = -0.1;  // 小さな負の値で後退
            } 
            else {
                // 既存の計算式を使用するが、最小距離を考慮
                msg.linear_velocity = std::min(distance2 - ((distance_opt / 2) + 0.15), 0.3);
            }

            msg.angle = Theta2;
            msg.formation_flag = true;
            ROS_INFO_STREAM(GREEN << "Publishing formation mode: flag=TRUE" << RESET);

            // デバッグ出力を追加
            ROS_INFO("Current objectpoint: %s at (%.2f, %.2f)", 
                latest_objectpoint_name_.c_str(), 
                latest_objectpoint_x_, 
                latest_objectpoint_y_);  

            formationModePub.publish(msg);
        } 
        else {
            msg.formation_flag = false;
            msg.angle = 0;
            msg.linear_velocity = 0;
            if (!has_received_data_) {
                ROS_ERROR("there is not near objectpoint");
            }
            ROS_INFO_STREAM(RED << "Publishing formation mode: flag=false (停止)" << RESET);
            formationModePub.publish(msg);
        }
    }
    else {
        // 追跡していない場合は停止命令を送信
        gnd_msgs::msg_Fformation_mode msg;
        msg.formation_flag = false;
        msg.angle = 0;
        msg.linear_velocity = 0;
        formationModePub.publish(msg);
    }

        loop_rate.sleep();
    }

    return 0;
}
