// sks_objectpoint_robot_person.cpp
// Legacy-behavior version (Mode B): pure distance gating (<=1.3m), no FOV, no hysteresis.
// Always face midpoint (human-objectpoint), 1.0s tracking timeout.
// Author: liurunzhi + gpt | License: BSD

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
#include <cmath>
#include <algorithm>

// If your message name differs, replace this include and type below accordingly.
#include <gnd_msgs/msg_Fformation_mode.h>  // fields: bool formation_flag; double angle; double linear_velocity

struct Pose2D
{
  double x{0.0}, y{0.0}, yaw{0.0};
  ros::Time stamp{};
};

class FFormationLegacy
{
public:
  FFormationLegacy(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh)
  {
    // --- Parameters (topic names & knobs) ---
    // Topic names can be overridden from launch.
    pnh_.param<std::string>("topic_person_pose", topic_person_pose_, "/pose_person_following");
    pnh_.param<std::string>("topic_robot_pose",  topic_robot_pose_,  "/pose_particle_localizer");
    pnh_.param<std::string>("topic_objectpoint", topic_objectpoint_, "/closest_objectpoint");
    pnh_.param<std::string>("topic_out_mode",    topic_out_mode_,    "/fformation_mode");

    // Control knobs (match legacy behavior)
    pnh_.param("timeout_tracking_sec", timeout_tracking_sec_, 1.0);  // legacy: 1.0s
    pnh_.param("enter_dist_max", enter_dist_max_, 1.3);              // legacy: 1.3m, no hysteresis
    pnh_.param("min_distance", min_distance_, 0.7);                  // stay-away belt center
    pnh_.param("dead_zone", dead_zone_, 0.3);                        // |distance2 - min_distance| < 0.3 => stop
    pnh_.param("v_linear_max", v_linear_max_, 0.3);                  // m/s
    pnh_.param("offset_halfspan", offset_halfspan_, 0.15);           // target belt offset: (d_opt/2)+0.15
    pnh_.param("quantize_dmin", quantize_dmin_, 0.50);               // human-objectpoint distance quantization range
    pnh_.param("quantize_dmax", quantize_dmax_, 1.80);
    pnh_.param("quantize_step", quantize_step_, 0.10);

    // --- Subscribers ---
    sub_person_ = nh_.subscribe(topic_person_pose_, 1, &FFormationLegacy::cbPerson, this);
    sub_robot_  = nh_.subscribe(topic_robot_pose_,  1, &FFormationLegacy::cbRobot,  this);
    sub_obj_    = nh_.subscribe(topic_objectpoint_, 1, &FFormationLegacy::cbObject, this);

    // --- Publisher ---
    pub_mode_ = nh_.advertise<gnd_msgs::msg_Fformation_mode>(topic_out_mode_, 1);

    // Loop
    double hz;
    pnh_.param("loop_hz", hz, 10.0);  // legacy used ~10Hz typical
    loop_rate_hz_ = std::max(1.0, hz);
  }

  void spin()
  {
    ros::Rate rate(loop_rate_hz_);
    while (ros::ok())
    {
      ros::spinOnce();
      step();
      rate.sleep();
    }
  }

private:
  // --- Callbacks ---
  void cbPerson(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    person_.x = msg->pose.position.x;
    person_.y = msg->pose.position.y;
    person_.yaw = yawFromQuat(msg->pose.orientation);
    person_.stamp = msg->header.stamp;
    got_person_ = true;
  }

  void cbRobot(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    robot_.x = msg->pose.position.x;
    robot_.y = msg->pose.position.y;
    robot_.yaw = yawFromQuat(msg->pose.orientation);
    robot_.stamp = msg->header.stamp;
    got_robot_ = true;
  }

  void cbObject(const geometry_msgs::PointStamped::ConstPtr& msg)
  {
    object_.x = msg->point.x;
    object_.y = msg->point.y;
    object_.stamp = msg->header.stamp;
    got_object_ = true;
  }

  // --- Main step (legacy hard gating) ---
  void step()
  {
    const ros::Time now = ros::Time::now();

    // 1) Preconditions: must have all data
    if (!got_robot_ || !got_person_ || !got_object_)
    {
      publishHalt();  // formation_flag = false
      return;
    }

    // 2) Tracking timeout on person pose (legacy: 1.0s)
    const double person_age = (now - person_.stamp).toSec();
    if (person_age > timeout_tracking_sec_)
    {
      publishHalt();
      return;
    }

    // 3) Compute human-objectpoint distance (d_opt) and quantize to [0.50, 1.80] step 0.10
    const double d_opt_raw = hypot(person_.x - object_.x, person_.y - object_.y);
    const double d_opt = quantizeClamp(d_opt_raw, quantize_dmin_, quantize_dmax_, quantize_step_);

    // 4) Legacy gating: enter only if d_opt <= 1.3m (no FOV, no hysteresis)
    const bool ok = (d_opt <= enter_dist_max_);
    if (!ok)
    {
      publishHalt();
      return;
    }

    // 5) Midpoint between human and objectpoint
    const double mid_x = 0.5 * (person_.x + object_.x);
    const double mid_y = 0.5 * (person_.y + object_.y);

    // 6) Angle to face the midpoint (Theta2), normalized to [-pi, pi]
    double theta2 = std::atan2(mid_y - robot_.y, mid_x - robot_.x) - robot_.yaw;
    theta2 = normalizeAngle(theta2);

    // 7) Distance from robot to the midpoint (distance2)
    const double distance2 = hypot(mid_x - robot_.x, mid_y - robot_.y);

    // 8) Target belt center: (d_opt/2) + 0.15
    const double target_dist = (d_opt * 0.5) + offset_halfspan_;

    // 9) Linear velocity with dead zone and gentle reverse
    double v = 0.0;
    if (std::fabs(distance2 - min_distance_) < dead_zone_)
    {
      // Inside dead-band: stop, keep facing theta2
      v = 0.0;
    }
    else if (distance2 < min_distance_)
    {
      // Too close to midpoint belt: slightly back off
      v = -0.1;
    }
    else
    {
      // Move forward toward the belt around midpoint
      v = std::min(distance2 - target_dist, v_linear_max_);
      v = std::max(v, 0.0);  // no forward neg speed
    }

    // 10) Publish formation mode (legacy: always face midpoint; angle=theta2)
    gnd_msgs::msg_Fformation_mode out;
    out.formation_flag = true;
    out.angle = theta2;
    out.linear_velocity = v;
    pub_mode_.publish(out);
  }

  // --- Helpers ---
  static double yawFromQuat(const geometry_msgs::Quaternion& q)
  {
    // Standard ZYX yaw extraction
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  static double normalizeAngle(double a)
  {
    while (a > M_PI)  a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  static double quantizeClamp(double v, double vmin, double vmax, double step)
  {
    // Clamp to [vmin, vmax] then quantize to nearest grid (e.g., 0.10m)
    double x = std::max(vmin, std::min(vmax, v));
    const double n = std::round((x - vmin) / step);
    return vmin + n * step;
  }

  void publishHalt()
  {
    gnd_msgs::msg_Fformation_mode out;
    out.formation_flag = false;
    out.angle = 0.0;
    out.linear_velocity = 0.0;
    pub_mode_.publish(out);
  }

private:
  ros::NodeHandle nh_, pnh_;

  // Params / knobs
  std::string topic_person_pose_;
  std::string topic_robot_pose_;
  std::string topic_objectpoint_;
  std::string topic_out_mode_;

  double timeout_tracking_sec_{1.0};
  double enter_dist_max_{1.3};
  double min_distance_{0.7};
  double dead_zone_{0.3};
  double v_linear_max_{0.3};
  double offset_halfspan_{0.15};
  double quantize_dmin_{0.50};
  double quantize_dmax_{1.80};
  double quantize_step_{0.10};
  double loop_rate_hz_{10.0};

  // State
  struct Obj2D { double x{0.0}, y{0.0}; ros::Time stamp{}; } object_;
  Pose2D person_, robot_;
  bool got_person_{false}, got_robot_{false}, got_object_{false};

  // ROS
  ros::Subscriber sub_person_;
  ros::Subscriber sub_robot_;
  ros::Subscriber sub_obj_;
  ros::Publisher  pub_mode_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sks_objectpoint_robot_person_legacy");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  FFormationLegacy node(nh, pnh);
  node.spin();
  return 0;
}
