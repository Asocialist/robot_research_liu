#include <cmath>
#include <limits>
#include <string>
#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "gnd_msgs/msg_pose2d_stamped.h"
#include "gnd_msgs/msg_waypoint_named.h"
#include "gnd_msgs/msg_Fformation_mode.h"

#ifndef M_PI
#define M_PI 3.14159265
#endif

// ===== Utils =====
static inline double normalizeAngle(double a){
  while(a >  M_PI) a -= 2*M_PI;
  while(a < -M_PI) a += 2*M_PI;
  return a;
}
static inline bool isObjectpointInPersonFOV(double px, double py, double pyaw,
                                            double ox, double oy, double deg){
  const double th = deg * M_PI / 180.0;
  double yaw_to_obj = std::atan2(oy - py, ox - px);
  return std::fabs(normalizeAngle(yaw_to_obj - pyaw)) <= th;
}

// ===== Config (可用 rosparam 配) =====
std::string topic_person = "pose_person_following";
std::string topic_robot  = "pose_particle_localizer";
std::string topic_object = "closest_objectpoint";
bool person_pose_is_global = false;  // 若 /pose_person_following 已是 map 系，设 true

// ===== States =====
struct Vec2d{ double x=0, y=0; };
Vec2d person, robot;
double robot_yaw=0, person_yaw=0;

bool has_robot=false, has_person=false;
ros::Time last_person_update;

std::string last_obj_name;
double obj_x=0, obj_y=0;
bool has_object=false;

// F-formation 状态保持
bool formation_active=false;

// 参数
double PERSON_TIMEOUT   = 3.0;  // 人物丢失超时 s
double ENTER_DIST_MAX   = 2.0;  // 进入阵距离
double ENTER_FOV_DEG    = 45.0; // 进入阵FOV
double HOLD_DIST_MAX    = 2.5;  // 保持阵距离
double HOLD_FOV_DEG     = 60.0; // 保持阵FOV
double MAX_V            = 0.3;  // 速度上限
double MIN_STOP_RADIUS  = 0.7;  // 近距离停止半径
double DEAD_ZONE        = 0.3;  // 死区

// ===== Callbacks =====
void robotCb(const gnd_msgs::msg_pose2d_stamped::ConstPtr& msg){
  robot.x = msg->x; robot.y = msg->y; robot_yaw = msg->theta;
  has_robot = true;
}
void personCb(const geometry_msgs::Pose2D::ConstPtr& m){
  if(person_pose_is_global){
    person.x = m->x; person.y = m->y; person_yaw = m->theta;
  }else{
    // 人物在 base_link，相对 -> map
    double c=std::cos(robot_yaw), s=std::sin(robot_yaw);
    person.x = robot.x + c*m->x - s*m->y;
    person.y = robot.y + s*m->x + c*m->y;
    person_yaw = normalizeAngle(robot_yaw + m->theta);
  }
  last_person_update = ros::Time::now();
  has_person = true;
}
void objCb(const gnd_msgs::msg_waypoint_named::ConstPtr& msg){
  last_obj_name = msg->name; obj_x = msg->x; obj_y = msg->y;
  has_object = true;
}

// ===== Main =====
int main(int argc, char** argv){
  ros::init(argc, argv, "robot_person_following_node");
  ros::NodeHandle nh("~");

  // params（可写到 launch 里）
  nh.param("topic_person", topic_person, topic_person);
  nh.param("topic_robot",  topic_robot,  topic_robot);
  nh.param("topic_object", topic_object, topic_object);
  nh.param("person_pose_is_global", person_pose_is_global, person_pose_is_global);

  nh.param("person_timeout",  PERSON_TIMEOUT,  PERSON_TIMEOUT);
  nh.param("enter_dist_max",  ENTER_DIST_MAX,  ENTER_DIST_MAX);
  nh.param("enter_fov_deg",   ENTER_FOV_DEG,   ENTER_FOV_DEG);
  nh.param("hold_dist_max",   HOLD_DIST_MAX,   HOLD_DIST_MAX);
  nh.param("hold_fov_deg",    HOLD_FOV_DEG,    HOLD_FOV_DEG);
  nh.param("max_v",           MAX_V,           MAX_V);
  nh.param("min_stop_radius", MIN_STOP_RADIUS, MIN_STOP_RADIUS);
  nh.param("dead_zone",       DEAD_ZONE,       DEAD_ZONE);

  auto pub = nh.advertise<gnd_msgs::msg_Fformation_mode>("formation_topic", 10);
  auto sr  = nh.subscribe(topic_robot,  100, robotCb);
  auto sp  = nh.subscribe(topic_person, 100, personCb);
  auto so  = nh.subscribe(topic_object, 10,  objCb);

  ros::Rate rate(10);
  while(ros::ok()){
    ros::spinOnce();
    gnd_msgs::msg_Fformation_mode cmd;  // angle, linear_velocity, formation_flag

    const bool person_ok = has_person &&
      ((ros::Time::now() - last_person_update).toSec() <= PERSON_TIMEOUT);

    if(person_ok && has_robot){
      // -------- 阵型判定 --------
      double dist_po = std::numeric_limits<double>::infinity();
      if(has_object){
        dist_po = std::hypot(obj_x - person.x, obj_y - person.y);
      }
      bool in_fov_enter = has_object &&
        isObjectpointInPersonFOV(person.x, person.y, person_yaw, obj_x, obj_y, ENTER_FOV_DEG);
      bool can_enter = has_object && (dist_po <= ENTER_DIST_MAX || in_fov_enter);

      bool in_fov_hold = has_object &&
        isObjectpointInPersonFOV(person.x, person.y, person_yaw, obj_x, obj_y, HOLD_FOV_DEG);
      bool can_hold  = has_object && (dist_po <= HOLD_DIST_MAX) && in_fov_hold;

      if(!formation_active && can_enter){
        formation_active = true;
        ROS_INFO_STREAM("[Formation] ENTER  dist="<<dist_po);
      }else if(formation_active && !can_hold){
        formation_active = false;
        ROS_WARN_STREAM("[Formation] EXIT   dist="<<dist_po);
      }

      if(formation_active){
        // —— 阵型：以“人-物”中点为目标 ——
        double midx = 0.5*(obj_x + person.x);
        double midy = 0.5*(obj_y + person.y);
        double dx = midx - robot.x, dy = midy - robot.y;
        double d  = std::hypot(dx,dy);
        double th = normalizeAngle(std::atan2(dy,dx) - robot_yaw);

        if(std::fabs(d - MIN_STOP_RADIUS) < DEAD_ZONE)       cmd.linear_velocity = 0.0;
        else if(d < MIN_STOP_RADIUS)                          cmd.linear_velocity = -0.1;
        else                                                  cmd.linear_velocity = std::min(d, MAX_V);

        cmd.angle = th;
        cmd.formation_flag = true;
      }else{
        // —— 纯追踪：以“人”为目标 ——
        double dx = person.x - robot.x, dy = person.y - robot.y;
        double d  = std::hypot(dx,dy);
        double th = normalizeAngle(std::atan2(dy,dx) - robot_yaw);

        if(std::fabs(d - MIN_STOP_RADIUS) < DEAD_ZONE)       cmd.linear_velocity = 0.0;
        else if(d < MIN_STOP_RADIUS)                          cmd.linear_velocity = -0.1;
        else                                                  cmd.linear_velocity = std::min(d, MAX_V);

        cmd.angle = th;
        cmd.formation_flag = false;  // 非阵型，仅追踪
      }
    }else{
      // 人物或机器人位姿不可用：停
      formation_active = false;
      cmd.linear_velocity = 0.0;
      cmd.angle = 0.0;
      cmd.formation_flag = false;
    }

    pub.publish(cmd);
    rate.sleep();
  }
  return 0;
}
