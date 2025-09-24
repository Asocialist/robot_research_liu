// people_detector_lidar.cpp
// Minimal LiDAR-based people detector via scan-line adjacency clustering.
// Author: liurunzhi
// License: BSD

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <cmath>
#include <limits>
#include <vector>
#include <string>
#include <algorithm>

struct Params {
  double min_range = 0.1;
  double max_range = 10.0;
  int    min_cluster_points = 3;
  int    max_cluster_points = 80;
  double cluster_dist_base = 0.10;   // base threshold (m)
  double cluster_dist_scale = 0.04;  // scaled by range: th = max(base, r*scale)
  double person_min_diameter = 0.20;
  double person_max_diameter = 0.80;
  int    downsample_step = 1;
  double marker_lifetime = 0.2;
  std::string frame_id; // leave empty to use scan frame
};

class PeopleDetectorLidar {
public:
  PeopleDetectorLidar(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh) {
    // Load params
    pnh_.param("min_range", params_.min_range, params_.min_range);
    pnh_.param("max_range", params_.max_range, params_.max_range);
    pnh_.param("min_cluster_points", params_.min_cluster_points, params_.min_cluster_points);
    pnh_.param("max_cluster_points", params_.max_cluster_points, params_.max_cluster_points);
    pnh_.param("cluster_dist_base", params_.cluster_dist_base, params_.cluster_dist_base);
    pnh_.param("cluster_dist_scale", params_.cluster_dist_scale, params_.cluster_dist_scale);
    pnh_.param("person_min_diameter", params_.person_min_diameter, params_.person_min_diameter);
    pnh_.param("person_max_diameter", params_.person_max_diameter, params_.person_max_diameter);
    pnh_.param("downsample_step", params_.downsample_step, params_.downsample_step);
    pnh_.param("marker_lifetime", params_.marker_lifetime, params_.marker_lifetime);
    pnh_.param("frame_id", params_.frame_id, params_.frame_id);

    sub_scan_ = nh_.subscribe("/scan", 1, &PeopleDetectorLidar::scanCb, this);
    pub_poses_ = nh_.advertise<geometry_msgs::PoseArray>("/people/poses", 1);
    pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("/people/markers", 1);
    pub_clusters_ = nh_.advertise<sensor_msgs::PointCloud>("/people/clusters", 1);

    ROS_INFO("[people_detector_lidar] ready.");
  }

private:
  // --- Utility: compute squared distance ---
  static inline double sqrDist(const geometry_msgs::Point32& a, const geometry_msgs::Point32& b) {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    return dx*dx + dy*dy;
  }

  // --- Convert a range+angle to XY point ---
  static inline bool polarToXY(double r, double angle, geometry_msgs::Point32& out) {
    if (!std::isfinite(r)) return false;
    out.x = r * std::cos(angle);
    out.y = r * std::sin(angle);
    out.z = 0.0;
    return true;
  }

  // --- Form clusters by adjacency in scan order ---
  void formClusters(const sensor_msgs::LaserScanConstPtr& scan,
                    std::vector<std::vector<geometry_msgs::Point32>>& clusters,
                    sensor_msgs::PointCloud* all_pts_opt) {
    clusters.clear();
    const int step = std::max(1, params_.downsample_step);
    geometry_msgs::Point32 prev_pt;
    bool has_prev = false;

    std::vector<geometry_msgs::Point32> current;

    const int n = static_cast<int>(scan->ranges.size());
    for (int i = 0; i < n; i += step) {
      const double r = scan->ranges[i];
      if (!std::isfinite(r) || r < params_.min_range || r > params_.max_range) {
        // If current cluster has content, flush it when we hit invalid
        if (!current.empty()) {
          clusters.emplace_back(current);
          current.clear();
        }
        has_prev = false;
        continue;
      }
      const double angle = scan->angle_min + i * scan->angle_increment;
      geometry_msgs::Point32 p;
      if (!polarToXY(r, angle, p)) continue;
      if (all_pts_opt) all_pts_opt->points.emplace_back(p);

      if (!has_prev) {
        current.clear();
        current.emplace_back(p);
        prev_pt = p;
        has_prev = true;
      } else {
        // adaptive threshold based on range
        const double th = std::max(params_.cluster_dist_base, r * params_.cluster_dist_scale);
        const double d2 = sqrDist(prev_pt, p);
        if (d2 <= th*th) {
          current.emplace_back(p);
        } else {
          // break cluster
          if (!current.empty()) clusters.emplace_back(current);
          current.clear();
          current.emplace_back(p);
        }
        prev_pt = p;
      }
    }
    // flush
    if (!current.empty()) clusters.emplace_back(current);
  }

  // --- Filter clusters to "person-like" by point count and spatial diameter ---
  bool isPersonLike(const std::vector<geometry_msgs::Point32>& cluster,
                    double& out_diameter) const {
    const int m = static_cast<int>(cluster.size());
    if (m < params_.min_cluster_points || m > params_.max_cluster_points) return false;

    // Estimate spatial diameter as max pairwise distance (O(m^2), but m is small)
    double max_d2 = 0.0;
    for (int i = 0; i < m; ++i) {
      for (int j = i+1; j < m; ++j) {
        const double d2 = sqrDist(cluster[i], cluster[j]);
        if (d2 > max_d2) max_d2 = d2;
      }
    }
    const double dia = std::sqrt(max_d2);
    out_diameter = dia;
    return (dia >= params_.person_min_diameter && dia <= params_.person_max_diameter);
  }

  // --- Compute centroid of a cluster ---
  static geometry_msgs::Point32 centroid(const std::vector<geometry_msgs::Point32>& cluster) {
    geometry_msgs::Point32 c; c.x = c.y = c.z = 0.0;
    if (cluster.empty()) return c;
    for (const auto& p : cluster) { c.x += p.x; c.y += p.y; }
    c.x /= cluster.size(); c.y /= cluster.size();
    return c;
  }

  // --- Marker helper ---
  visualization_msgs::Marker makeMarker(const geometry_msgs::Point32& c,
                                        int id,
                                        const std::string& ns,
                                        const std::string& frame,
                                        double lifetime) const {
    visualization_msgs::Marker mk;
    mk.header.stamp = ros::Time::now();
    mk.header.frame_id = frame;
    mk.ns = ns;
    mk.id = id;
    mk.type = visualization_msgs::Marker::CYLINDER;
    mk.action = visualization_msgs::Marker::ADD;

    mk.pose.position.x = c.x;
    mk.pose.position.y = c.y;
    mk.pose.position.z = 0.0;
    mk.pose.orientation.w = 1.0;

    mk.scale.x = 0.35; // cylinder diameter (m)
    mk.scale.y = 0.35;
    mk.scale.z = 1.2;  // height (m)

    mk.color.r = 0.1f;
    mk.color.g = 0.8f;
    mk.color.b = 0.2f;
    mk.color.a = 0.7f;

    mk.lifetime = ros::Duration(lifetime);
    return mk;
  }

  void scanCb(const sensor_msgs::LaserScanConstPtr& scan) {
    const std::string frame = params_.frame_id.empty() ? scan->header.frame_id : params_.frame_id;

    // Optional debug cloud
    sensor_msgs::PointCloud all_pts;
    all_pts.header.stamp = scan->header.stamp;
    all_pts.header.frame_id = frame;

    // Step 1: clustering
    std::vector<std::vector<geometry_msgs::Point32>> clusters;
    formClusters(scan, clusters, pub_clusters_.getNumSubscribers() ? &all_pts : nullptr);

    if (pub_clusters_.getNumSubscribers()) {
      pub_clusters_.publish(all_pts);
    }

    // Step 2: filter to person-like and build outputs
    geometry_msgs::PoseArray poses;
    poses.header.stamp = scan->header.stamp;
    poses.header.frame_id = frame;

    visualization_msgs::MarkerArray markers;
    // clear old markers by publishing 0 lifetime delete-all? Simpler: reuse ids and short lifetime.

    int id = 0;
    for (const auto& cl : clusters) {
      double dia = 0.0;
      if (!isPersonLike(cl, dia)) continue;
      const auto c = centroid(cl);

      geometry_msgs::Pose p;
      p.position.x = c.x;
      p.position.y = c.y;
      p.position.z = 0.0;
      p.orientation.w = 1.0;
      poses.poses.emplace_back(p);

      markers.markers.emplace_back(makeMarker(c, id++, "people", frame, params_.marker_lifetime));
    }

    // Publish
    pub_poses_.publish(poses);
    if (!markers.markers.empty() || pub_markers_.getNumSubscribers()) {
      pub_markers_.publish(markers);
    }
  }

private:
  ros::NodeHandle nh_, pnh_;
  Params params_;
  ros::Subscriber sub_scan_;
  ros::Publisher pub_poses_;
  ros::Publisher pub_markers_;
  ros::Publisher pub_clusters_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "people_detector_lidar");
  ros::NodeHandle nh, pnh("~");
  PeopleDetectorLidar node(nh, pnh);
  ros::spin();
  return 0;
}
