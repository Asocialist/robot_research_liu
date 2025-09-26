// liu_fformation_node.cpp
// Author: liu 
// Purpose: Track -> detect group entry -> compute F-formation pose -> enqueue via AddDestinationPose into nkm_destination_queue

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <vector>

// Queue service (as defined in your updated destination queue)
#include <nkm_destination_queue/AddDestinationPose.h>  // Request: geometry_msgs::PoseStamped pose; indexInQueue enum
                                                       // Response: int32 orderID

// ---------------- Params ----------------
struct Params {
  std::string map_frame = "map";
  std::string people_pose_topic = "/people/poses"; // geometry_msgs::PoseArray (group detections)
  std::string robot_odom_topic  = "/odom";         // nav_msgs::Odometry

  int    min_group_size      = 2;    // minimum number of detections to constitute a group
  double max_group_radius_m  = 1.5;  // maximum radius (m) from centroid to keep group tight

  // Group detection thresholds
  double group_radius_m      = 1.6;   // distance threshold to consider "in group"
  double group_stable_time_s = 0.8;   // how long the condition must hold
  double loss_timeout_s      = 2.5;   // target lost timeout

  // F-formation geometry
  double f_distance_m  = 0.9;         // distance in front of human (along human yaw)
  double f_angle_deg   = 0.0;         // optional lateral offset angle (0 means straight ahead)

  // Send throttle
  double resend_interval_s = 1.0;     // do not enqueue more than once per this period
};

// ---------------- State Machine ----------------
enum class State {
  TRACKING = 0,
  IN_GROUP = 1,
  F_FORMATION = 2
};

class LiuFFormationNode {
public:
  explicit LiuFFormationNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh), tf_buffer_(ros::Duration(10.0)), tf_listener_(tf_buffer_) {
    // Load params
    pnh_.param("map_frame", params_.map_frame, params_.map_frame);
    pnh_.param("people_pose_topic", params_.people_pose_topic, params_.people_pose_topic);
    pnh_.param("robot_odom_topic",  params_.robot_odom_topic,  params_.robot_odom_topic);
    pnh_.param<int>("min_group_size", params_.min_group_size, params_.min_group_size);
    pnh_.param("max_group_radius_m", params_.max_group_radius_m, params_.max_group_radius_m);

    pnh_.param("group_radius_m",      params_.group_radius_m,      params_.group_radius_m);
    pnh_.param("group_stable_time_s", params_.group_stable_time_s, params_.group_stable_time_s);
    pnh_.param("loss_timeout_s",      params_.loss_timeout_s,      params_.loss_timeout_s);

    pnh_.param("f_distance_m",  params_.f_distance_m,  params_.f_distance_m);
    pnh_.param("f_angle_deg",   params_.f_angle_deg,   params_.f_angle_deg);

    pnh_.param("resend_interval_s", params_.resend_interval_s, params_.resend_interval_s);

    // Subscribers
    sub_people_ = nh_.subscribe(params_.people_pose_topic, 1, &LiuFFormationNode::onPeoplePoses, this);
    sub_odom_   = nh_.subscribe(params_.robot_odom_topic,  1, &LiuFFormationNode::onOdom, this);

    // Service client to add pose destination (queue head for immediate effect)
    // Name and type follow your updated queue node (defSrvNameAddDestPose)
    srv_add_pose_ = nh_.serviceClient<nkm_destination_queue::AddDestinationPose>(
        "nkm_destination_queue/add_destination_pose"); // matches your file
    // Note: The server is started by the destination queue node. We can wait or just try-call.

    last_state_change_ = ros::Time::now();
    last_group_time_   = ros::Time(0);
    last_sent_time_    = ros::Time(0);

    ROS_INFO("[liu_fformation_node] ready. Using service: nkm_destination_queue/add_destination_pose");
  }

  void spin() {
    ros::Rate r(30);
    while (ros::ok()) {
      ros::spinOnce();
      step();
      r.sleep();
    }
  }

private:
  // ---------- Callbacks ----------
  void onPeoplePoses(const geometry_msgs::PoseArray::ConstPtr& msg) {
    latest_people_ = *msg;
    ros::Time stamp = msg->header.stamp;
    if (stamp.isZero()) {
      stamp = ros::Time::now();
    }

    geometry_msgs::PoseStamped group_pose;
    if (computeGroupPose(latest_people_, group_pose)) {
      latest_group_pose_ = group_pose;
      has_group_pose_ = true;
      last_group_time_ = stamp;
    } else {
      has_group_pose_ = false;
      last_group_time_ = stamp;
    }
  }

  void onOdom(const nav_msgs::Odometry::ConstPtr& msg) {
    latest_odom_ = *msg;
  }

  bool computeGroupPose(const geometry_msgs::PoseArray& arr, geometry_msgs::PoseStamped& out) {
    if (arr.poses.size() < static_cast<std::size_t>(params_.min_group_size)) {
      last_group_size_ = arr.poses.size();
      last_group_spread_ = 0.0;
      return false;
    }

    ros::Time stamp = arr.header.stamp;
    if (stamp.isZero()) {
      stamp = ros::Time::now();
    }

    const std::string source_frame = arr.header.frame_id.empty() ? params_.map_frame : arr.header.frame_id;

    std::vector<geometry_msgs::Point> points;
    points.reserve(arr.poses.size());

    for (const auto& pose : arr.poses) {
      geometry_msgs::PoseStamped pose_stamped;
      pose_stamped.header.frame_id = source_frame;
      pose_stamped.header.stamp = stamp;
      pose_stamped.pose = pose;

      geometry_msgs::PoseStamped transformed;
      if (!transformToMap(pose_stamped, transformed)) {
        continue;
      }
      geometry_msgs::Point pt;
      pt.x = transformed.pose.position.x;
      pt.y = transformed.pose.position.y;
      pt.z = transformed.pose.position.z;
      points.emplace_back(pt);
    }

    if (points.size() < static_cast<std::size_t>(params_.min_group_size)) {
      last_group_size_ = points.size();
      last_group_spread_ = 0.0;
      return false;
    }

    double cx = 0.0;
    double cy = 0.0;
    for (const auto& pt : points) {
      cx += pt.x;
      cy += pt.y;
    }
    cx /= static_cast<double>(points.size());
    cy /= static_cast<double>(points.size());

    double max_radius = 0.0;
    for (const auto& pt : points) {
      const double dx = pt.x - cx;
      const double dy = pt.y - cy;
      const double r = std::sqrt(dx * dx + dy * dy);
      if (r > max_radius) {
        max_radius = r;
      }
    }

    last_group_size_ = points.size();
    last_group_spread_ = max_radius;

    if (params_.max_group_radius_m > 0.0 && max_radius > params_.max_group_radius_m) {
      ROS_DEBUG_THROTTLE(2.0,
                         "[liu_fformation_node] group spread %.2f exceeds max %.2f (count=%zu)",
                         max_radius, params_.max_group_radius_m, last_group_size_);
      return false;
    }

    out.header.frame_id = params_.map_frame;
    out.header.stamp = stamp;
    out.pose.position.x = cx;
    out.pose.position.y = cy;
    out.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
    out.pose.orientation = tf2::toMsg(q);

    return true;
  }

  bool transformToMap(const geometry_msgs::PoseStamped& in, geometry_msgs::PoseStamped& out) {
    if (in.header.frame_id.empty() || in.header.frame_id == params_.map_frame) {
      out = in;
      out.header.frame_id = params_.map_frame;
      return true;
    }

    try {
      geometry_msgs::TransformStamped tf =
          tf_buffer_.lookupTransform(params_.map_frame, in.header.frame_id, in.header.stamp, ros::Duration(0.05));
      tf2::doTransform(in, out, tf);
      return true;
    } catch (const tf2::TransformException& ex) {
      ROS_WARN_THROTTLE(2.0,
                        "[liu_fformation_node] transform failed %s -> %s: %s",
                        in.header.frame_id.c_str(),
                        params_.map_frame.c_str(),
                        ex.what());
      return false;
    }
  }

  // ---------- FSM ----------
  void step() {
    const ros::Time now = ros::Time::now();
    const bool group_recent = has_group_pose_ && ((now - last_group_time_).toSec() < params_.loss_timeout_s);

    if (!group_recent) {
      if (state_ != State::TRACKING) transitionTo(State::TRACKING, "group lost");
      return;
    }

    const double dist = robotToGroupDistance();
    const bool in_group_cond = std::isfinite(dist) && (dist <= params_.group_radius_m);

    switch (state_) {
      case State::TRACKING: {
        if (in_group_cond) {
          if (!candidate_since_.isZero()) {
            if ((now - candidate_since_).toSec() >= params_.group_stable_time_s) {
              transitionTo(State::IN_GROUP, "group proximity stable");
            }
          } else {
            candidate_since_ = now;
          }
        } else {
          candidate_since_ = ros::Time(0);
        }
        break;
      }

      case State::IN_GROUP: {
        if (!in_group_cond) {
          transitionTo(State::TRACKING, "left group vicinity");
          break;
        }
        if (!has_group_pose_) {
          transitionTo(State::TRACKING, "group pose invalidated");
          break;
        }
        // Compute and enqueue F-formation once, then switch
        geometry_msgs::PoseStamped goal = computeFFormation(latest_group_pose_);
        tryEnqueueGoal(goal);
        transitionTo(State::F_FORMATION, "goal enqueued, entering F_FORMATION");
        break;
      }

      case State::F_FORMATION: {
        if (!in_group_cond) {
          transitionTo(State::TRACKING, "left group vicinity");
          break;
        }
        if (!has_group_pose_) {
          transitionTo(State::TRACKING, "group pose invalidated");
          break;
        }
        if (needResend(now)) {
          geometry_msgs::PoseStamped goal = computeFFormation(latest_group_pose_);
          tryEnqueueGoal(goal);
        }
        break;
      }
    }
  }

  void transitionTo(State s, const char* reason) {
    state_ = s;
    last_state_change_ = ros::Time::now();
    candidate_since_ = ros::Time(0);
    ROS_INFO("[liu_fformation_node] transition -> %d (%s) group_size=%zu spread=%.2f",
             static_cast<int>(s), reason, last_group_size_, last_group_spread_);
  }

  // ---------- Geometry ----------
  double robotToGroupDistance() const {
    const double rx = latest_odom_.pose.pose.position.x;
    const double ry = latest_odom_.pose.pose.position.y;
    const double gx = latest_group_pose_.pose.position.x;
    const double gy = latest_group_pose_.pose.position.y;
    const double dx = gx - rx;
    const double dy = gy - ry;
    return std::sqrt(dx * dx + dy * dy);
  }

  static double yawFromQuat(const geometry_msgs::Quaternion& q) {
    tf2::Quaternion tq;
    tf2::fromMsg(q, tq);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tq).getRPY(roll, pitch, yaw);
    return yaw;
  }

  geometry_msgs::PoseStamped computeFFormation(const geometry_msgs::PoseStamped& group) const {
    geometry_msgs::PoseStamped out;
    out.header.stamp = ros::Time::now();
    out.header.frame_id = params_.map_frame;

    const double gx = group.pose.position.x;
    const double gy = group.pose.position.y;

    const double rx = latest_odom_.pose.pose.position.x;
    const double ry = latest_odom_.pose.pose.position.y;

    double dir_x = rx - gx;
    double dir_y = ry - gy;
    double norm = std::hypot(dir_x, dir_y);

    if (norm < 1e-3) {
      const double yaw = yawFromQuat(latest_odom_.pose.pose.orientation);
      dir_x = std::cos(yaw);
      dir_y = std::sin(yaw);
      norm = 1.0;
    }

    dir_x /= norm;
    dir_y /= norm;

    const double offset = params_.f_angle_deg * M_PI / 180.0;
    const double cos_off = std::cos(offset);
    const double sin_off = std::sin(offset);

    const double adj_x = cos_off * dir_x - sin_off * dir_y;
    const double adj_y = sin_off * dir_x + cos_off * dir_y;

    out.pose.position.x = gx + params_.f_distance_m * adj_x;
    out.pose.position.y = gy + params_.f_distance_m * adj_y;
    out.pose.position.z = 0.0;

    const double yaw_r = std::atan2(gy - out.pose.position.y, gx - out.pose.position.x);

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_r);
    out.pose.orientation = tf2::toMsg(q);
    return out;
  }

  // ---------- Service enqueue ----------
  bool needResend(const ros::Time& now) const {
    return (now - last_sent_time_).toSec() >= params_.resend_interval_s;
  }

  void tryEnqueueGoal(const geometry_msgs::PoseStamped& goal) {
    const ros::Time now = ros::Time::now();
    if (!needResend(now)) return;

    nkm_destination_queue::AddDestinationPose srv;
    srv.request.pose = goal;
    // Insert at queue head to take effect immediately (matches your queue semantics)
    srv.request.indexInQueue = nkm_destination_queue::AddDestinationPose::Request::Current;

    if (!srv_add_pose_.exists()) {
      ROS_WARN_THROTTLE(2.0, "[liu_fformation_node] service not available yet: nkm_destination_queue/add_destination_pose");
    }

    if (srv_add_pose_.call(srv)) {
      last_sent_time_ = now;
      ROS_INFO("[liu_fformation_node] enqueued F-formation pose (orderID=%d).", srv.response.orderID);
    } else {
      ROS_ERROR("[liu_fformation_node] AddDestinationPose call failed.");
    }
  }

private:
  ros::NodeHandle nh_, pnh_;
  Params params_;
  State state_{State::TRACKING};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::Subscriber sub_people_;
  ros::Subscriber sub_odom_;
  ros::ServiceClient srv_add_pose_;

  nav_msgs::Odometry latest_odom_;
  geometry_msgs::PoseArray latest_people_;
  geometry_msgs::PoseStamped latest_group_pose_;

  bool has_group_pose_{false};
  std::size_t last_group_size_{0};
  double last_group_spread_{0.0};

  ros::Time last_state_change_;
  ros::Time candidate_since_;
  ros::Time last_group_time_;
  ros::Time last_sent_time_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "liu_fformation_node");
  ros::NodeHandle nh, pnh("~");
  LiuFFormationNode node(nh, pnh);
  node.spin();
  return 0;
}
