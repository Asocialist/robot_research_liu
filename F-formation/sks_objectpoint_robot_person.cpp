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

// ===== 角度&朝向工具函数 =====
static inline double normalizeAngle(double a) {
    while (a > M_PI)  a -= 2*M_PI;
    while (a < -M_PI) a += 2*M_PI;
    return a;
}

// 判断“人物朝向 personYaw”和“人物→objectpoint 方向角”夹角是否 ≤ deg（默认45°）
static inline bool isObjectpointInPersonFOV(double personX, double personY, double personYaw,
                                            double objX, double objY, double deg = 45.0) {
    const double dx = objX - personX;
    const double dy = objY - personY;
    const double targetYaw = atan2(dy, dx);        // 人物→objectpoint 的方向角
    double diff = normalizeAngle(targetYaw - personYaw);
    const double th = deg * M_PI / 180.0;          // 阈值（弧度）
    return std::abs(diff) <= th;
}
// ==================================

// ---Configure---
std::string defNodeName = "sks_objectpoint_robot_person";
std::string defTopicNameTrackingPosition = "pose_person_following";
std::string defTopicNameVehiclePose = "pose_particle_localizer";
std::string defPathFileName = "/home/liu/liu_workplace/ros/locations/250725/250725.path";

double defRediscoverySecRate = 2.0;
double defDistanceArrivePoint = 0.15;
bool   defIsRegistDestinaionAuto = true;
bool   defIsTrackingPositionGlobal = false;

// ---Variables---
Vec2d_t poseTracking = Vec2d_t(0.0, 0.0);
Vec2d_t poseVehicle  = Vec2d_t(0.0, 0.0);
double  poseVehicleT = 0;
double  poseTrackingT = 0;

ros::Time timeLastRediscovery;

bool isReceivedrobotpose = false;

bool isReceivedLatestTracking = false;
bool isMovingToTracking = false;
Vec2d_t pointTrackingOnPath;
int indexDestQueueOrder = 0;

std::string latest_objectpoint_name_;
double latest_objectpoint_x_;
double latest_objectpoint_y_;
bool has_received_data_ = false;

// sbtpへ
ros::Publisher formationModePub;

// 追踪状态
struct TrackingStatus {
    ros::Time lastUpdateTime;
    bool isTracking;
    static constexpr double TIMEOUT_DURATION = 3.0;  // ★ 放宽为3s
} trackingStatus;

// ---“松弛进入/保持”参数---
static constexpr double ENTER_DIST_MAX   = 2.0;   // ★ 进入：距离 ≤ 2.0 m
static constexpr double ENTER_FOV_DEG    = 45.0;  // ★ 进入：FOV ±45°
static constexpr double HOLD_DIST_MAX    = 2.5;   // ★ 保持：距离 ≤ 2.5 m（迟滞）
static constexpr double HOLD_FOV_DEG     = 60.0;  // ★ 保持：FOV ±60°
static bool formation_active = false;             // 阵型状态保持位

// objectpoint 回调
void closestObjectpointCallback(const gnd_msgs::msg_waypoint_named::ConstPtr& msg) {
    if (!trackingStatus.isTracking) {
        // 未追踪时不更新
        return;
    }
    latest_objectpoint_name_ = msg->name;
    latest_objectpoint_x_ = msg->x;
    latest_objectpoint_y_ = msg->y;
    has_received_data_ = true;

    ROS_WARN("Received objectpoint: %s at (%.2f, %.2f)",
             msg->name.c_str(), msg->x, msg->y);
}

// 人物位姿回调
void personPositionCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    ros::Time currentTime = ros::Time::now();

    if(defIsTrackingPositionGlobal){
        poseTracking.x = msg->x;
        poseTracking.y = msg->y;
        poseTrackingT  = msg->theta;  // 全局朝向
    } else {
        // 局部->map
        poseTracking.x = msg->x * cos(poseVehicleT) - msg->y * sin(poseVehicleT) + poseVehicle.x;
        poseTracking.y = msg->x * sin(poseVehicleT) + msg->y * cos(poseVehicleT) + poseVehicle.y;
        poseTrackingT  = normalizeAngle(msg->theta + poseVehicleT);
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

    // ---ROS 初始化---
    ros::init(argc, argv, defNodeName.c_str());
    ros::NodeHandle nodeHandle;

    formationModePub = nodeHandle.advertise<gnd_msgs::msg_Fformation_mode>("formation_topic", 10);

    // 机器人位姿
    ros::Subscriber subVehiclePosition = nodeHandle.subscribe(
        defTopicNameVehiclePose.c_str(), 100,
        +[](const gnd_msgs::msg_pose2d_stamped::ConstPtr& msg) {
            poseVehicle.x = msg->x;
            poseVehicle.y = msg->y;
            poseVehicleT  = msg->theta;
            isReceivedrobotpose = true;
            return;
        });

    // 人物位姿
    ros::Subscriber subPersonPosition = nodeHandle.subscribe(
        defTopicNameTrackingPosition.c_str(), 100, personPositionCallback);

    // 最近objectpoint
    ros::Subscriber sub = nodeHandle.subscribe("closest_objectpoint", 10, closestObjectpointCallback);

    // ---Main Loop---
    timeLastRediscovery = ros::Time::now();
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        ros::spinOnce();

        // 追踪状态更新/超时（★ 3s）
        ros::Time currentTime = ros::Time::now();
        if ((currentTime - trackingStatus.lastUpdateTime).toSec() > TrackingStatus::TIMEOUT_DURATION) {
            if (trackingStatus.isTracking) {
                ROS_WARN("Lost person tracking (>3s)");
                trackingStatus.isTracking = false;
                has_received_data_ = false;   // objectpoint 失效
                formation_active = false;     // 阵型失效
            }
        }

        if (trackingStatus.isTracking && isReceivedrobotpose) {
            gnd_msgs::msg_Fformation_mode msg;

            // 人—objectpoint 距离
            double distance_opt = std::numeric_limits<double>::infinity();
            if (has_received_data_) {
                distance_opt = std::hypot(latest_objectpoint_x_ - poseTracking.x,
                                          latest_objectpoint_y_ - poseTracking.y);
            }

            // FOV 判定
            const bool in_person_fov = has_received_data_ ?
                isObjectpointInPersonFOV(poseTracking.x, poseTracking.y, poseTrackingT,
                                         latest_objectpoint_x_, latest_objectpoint_y_,
                                         ENTER_FOV_DEG) : false;

            // ------- 松弛进入逻辑（减少重复约束） -------
            // 条件：1) 有 objectpoint 数据
            //       2) （距离≤2.0m）或（FOV±45°）满足其一即可进入
            bool can_enter = has_received_data_ && (distance_opt <= ENTER_DIST_MAX || in_person_fov);

            // 迟滞保持：一旦进入，放宽到 距离≤2.5m 且 FOV±60°
            bool can_hold  = has_received_data_ && (distance_opt <= HOLD_DIST_MAX) &&
                             isObjectpointInPersonFOV(poseTracking.x, poseTracking.y, poseTrackingT,
                                                      latest_objectpoint_x_, latest_objectpoint_y_,
                                                      HOLD_FOV_DEG);

            if (!formation_active && can_enter) {
                formation_active = true;
                ROS_INFO(GREEN "Fformation ENTER (relaxed): dist=%.2f, FOV<=%.0f" RESET,
                         distance_opt, ENTER_FOV_DEG);
            } else if (formation_active && !can_hold) {
                formation_active = false;
                ROS_WARN(YELLOW "Fformation EXIT (hold failed): dist=%.2f / FOV>%.0f" RESET,
                         distance_opt, HOLD_FOV_DEG);
            }

            if (formation_active) {
                // —— 速度与角度控制（沿用你的量化逻辑） ——
                if (std::isinf(distance_opt)) {
                    // 极少数 race 条件：进入后一帧丢objectpoint
                    distance_opt = 1.0;
                }

                // 离散化期望距离（原逻辑保留）
                if (distance_opt >= 0.01 && distance_opt < 0.5)      distance_opt = 0.50;
                else if (distance_opt < 0.6)                         distance_opt = 0.60;
                else if (distance_opt < 0.7)                         distance_opt = 0.70;
                else if (distance_opt < 0.8)                         distance_opt = 0.80;
                else if (distance_opt < 0.9)                         distance_opt = 0.90;
                else if (distance_opt < 1.0)                         distance_opt = 1.00;
                else if (distance_opt < 1.1)                         distance_opt = 1.10;
                else if (distance_opt < 1.2)                         distance_opt = 1.20;
                else if (distance_opt < 1.3)                         distance_opt = 1.30;
                else if (distance_opt < 1.4)                         distance_opt = 1.40;
                else if (distance_opt < 1.5)                         distance_opt = 1.50;
                else if (distance_opt < 1.6)                         distance_opt = 1.60;
                else if (distance_opt < 1.7)                         distance_opt = 1.70;
                else if (distance_opt < 1.8)                         distance_opt = 1.80;

                // 取“人物—objectpoint”的中点为目标
                double midX = (latest_objectpoint_x_ + poseTracking.x) / 2.0;
                double midY = (latest_objectpoint_y_ + poseTracking.y) / 2.0;

                // 机器人到中点的向量和方位
                double diffX = midX - poseVehicle.x;
                double diffY = midY - poseVehicle.y;
                double distance2 = std::hypot(diffX, diffY);
                double Theta2 = normalizeAngle(std::atan2(diffY, diffX) - poseVehicleT);

                // 最小距离/死区
                double min_distance = 0.7;
                const double DEAD_ZONE = 0.3;

                if (std::abs(distance2 - min_distance) < DEAD_ZONE) {
                    msg.linear_velocity = 0.0;              // 死区内停止
                } else if (distance2 < min_distance) {
                    msg.linear_velocity = -0.1;             // 太近后退
                } else {
                    msg.linear_velocity = std::min(distance2 - ((distance_opt / 2) + 0.15), 0.3);
                }

                msg.angle = Theta2;
                msg.formation_flag = true;

                ROS_INFO_STREAM(GREEN << "Publishing formation mode: flag=TRUE"
                               << " dist2=" << distance2 << " angle=" << msg.angle << RESET);

                formationModePub.publish(msg);
            } else {
                // 不满足：停
                gnd_msgs::msg_Fformation_mode halt;
                halt.formation_flag = false;
                halt.angle = 0;
                halt.linear_velocity = 0;

                if (!has_received_data_) {
                    ROS_ERROR("No objectpoint yet.");
                }

                // 仅在还未进入时提示进入约束（避免刷屏）
                if (!formation_active) {
                    if (has_received_data_ && !in_person_fov && distance_opt > ENTER_DIST_MAX) {
                        ROS_WARN("Blocked: need dist<=%.1fm OR FOV<=±%.0fdeg (curr dist=%.2f)",
                                ENTER_DIST_MAX, ENTER_FOV_DEG, distance_opt);
                    }
                }

                formationModePub.publish(halt);
            }
        } else {
            // 追踪/机器人位姿不可用：保持停止
            gnd_msgs::msg_Fformation_mode msg;
            msg.formation_flag = false;
            msg.angle = 0;
            msg.linear_velocity = 0;
            formation_active = false;
            formationModePub.publish(msg);
        }

        loop_rate.sleep();
    }

    return 0;
}
