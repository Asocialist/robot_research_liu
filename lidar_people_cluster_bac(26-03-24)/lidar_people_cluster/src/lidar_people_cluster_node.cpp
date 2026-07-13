#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>

#include <std_msgs/Int32MultiArray.h>

#include <visualization_msgs/MarkerArray.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <unordered_map>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

struct PointXYZ { float x; float y; float z; };

static inline bool isFiniteFloat(float v) { return std::isfinite(v); }
static inline uint64_t packCellKey(int cx, int cy) {
  return (static_cast<uint64_t>(static_cast<uint32_t>(cx)) << 32) |
         (static_cast<uint64_t>(static_cast<uint32_t>(cy)));
}
static inline void hsvToRgb(float h, float s, float v, float &r, float &g, float &b) {
  float c = v * s;
  float hp = h * 6.0f;
  float x = c * (1.0f - std::fabs(std::fmod(hp, 2.0f) - 1.0f));
  float m = v - c;
  float rp = 0, gp = 0, bp = 0;
  if (0.0f <= hp && hp < 1.0f) { rp = c; gp = x; bp = 0; }
  else if (1.0f <= hp && hp < 2.0f) { rp = x; gp = c; bp = 0; }
  else if (2.0f <= hp && hp < 3.0f) { rp = 0; gp = c; bp = x; }
  else if (3.0f <= hp && hp < 4.0f) { rp = 0; gp = x; bp = c; }
  else if (4.0f <= hp && hp < 5.0f) { rp = x; gp = 0; bp = c; }
  else { rp = c; gp = 0; bp = x; }
  r = rp + m; g = gp + m; b = bp + m;
}

static inline double normalizeAngle(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

static inline double angleDiff(double a, double b) {
  return normalizeAngle(a - b);
}

// Estimate ellipse parameters from a cluster.
// - ellipse_yaw: major-axis yaw (base_frame, rad)
// - major_d/minor_d: diameters along major/minor axis (meters)
static inline void estimateEllipseFromCluster(
    const std::vector<PointXYZ> &pts,
    const std::vector<int> &idx,
    double cx, double cy,
    double &ellipse_yaw,
    double &major_d,
    double &minor_d)
{
  const int n = (int)idx.size();
  if (n < 3) {
    ellipse_yaw = 0.0;
    major_d = 0.3;
    minor_d = 0.2;
    return;
  }

  // Covariance in XY around centroid.
  double sxx = 0.0, sxy = 0.0, syy = 0.0;
  for (int k : idx) {
    const double dx = (double)pts[k].x - cx;
    const double dy = (double)pts[k].y - cy;
    sxx += dx * dx;
    sxy += dx * dy;
    syy += dy * dy;
  }
  sxx /= (double)n;
  sxy /= (double)n;
  syy /= (double)n;

  // Principal direction (major axis).
  ellipse_yaw = 0.5 * std::atan2(2.0 * sxy, (sxx - syy));
  ellipse_yaw = normalizeAngle(ellipse_yaw);

  // Project points to major/minor axis to get extents.
  const double c = std::cos(ellipse_yaw);
  const double s = std::sin(ellipse_yaw);

  double umin = 1e9, umax = -1e9;
  double vmin = 1e9, vmax = -1e9;

  for (int k : idx) {
    const double dx = (double)pts[k].x - cx;
    const double dy = (double)pts[k].y - cy;
    const double u = dx * c + dy * s;     // along major
    const double v = -dx * s + dy * c;    // along minor
    umin = std::min(umin, u); umax = std::max(umax, u);
    vmin = std::min(vmin, v); vmax = std::max(vmax, v);
  }

  major_d = std::max(0.15, std::min(1.20, (umax - umin)));
  minor_d = std::max(0.10, std::min(1.00, (vmax - vmin)));
}


class LidarPeopleClusterNode {
public:
  LidarPeopleClusterNode()
  : nh_(), pnh_("~"), tf_buffer_(), tf_listener_(tf_buffer_) {
    pnh_.param<std::string>("scan1_topic", scan1_topic_, "/scan1");
    pnh_.param<std::string>("scan2_topic", scan2_topic_, "/scan2");
    pnh_.param<std::string>("base_frame", base_frame_, "base_link");

    pnh_.param<double>("range_min", range_min_, 0.05);
    pnh_.param<double>("range_max", range_max_, 12.0);

    pnh_.param<double>("cluster_eps", cluster_eps_, 0.15);
    pnh_.param<int>("cluster_min_pts", cluster_min_pts_, 8);
    pnh_.param<int>("cluster_max_pts", cluster_max_pts_, 200);

    pnh_.param<double>("cluster_min_width", cluster_min_width_, 0.20);
    pnh_.param<double>("cluster_max_width", cluster_max_width_, 0.90);

    pnh_.param<double>("candidate_min_dist", candidate_min_dist_, 0.3);
    pnh_.param<double>("candidate_max_dist", candidate_max_dist_, 6.0);

    pnh_.param<double>("track_gate_dist", track_gate_dist_, 0.60);
    pnh_.param<double>("track_timeout", track_timeout_, 0.8);

    pnh_.param<bool>("publish_fused_cloud", publish_fused_cloud_, true);
    pnh_.param<double>("theta_offset", theta_offset_, 0.0);


    pub_candidates_ = nh_.advertise<geometry_msgs::PoseArray>("/people_candidates", 1);
    pub_tracks_     = nh_.advertise<geometry_msgs::PoseArray>("/people_tracks", 1);
    pub_track_ids_  = nh_.advertise<std_msgs::Int32MultiArray>("/people_track_ids", 1);
    pub_markers_    = nh_.advertise<visualization_msgs::MarkerArray>("/people_markers", 1);
    pub_cloud_      = nh_.advertise<sensor_msgs::PointCloud2>("/scan_fused_cloud", 1);

    sub_scan1_.reset(new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, scan1_topic_, 5));
    sub_scan2_.reset(new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, scan2_topic_, 5));
    sync_.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), *sub_scan1_, *sub_scan2_));
    sync_->registerCallback(boost::bind(&LidarPeopleClusterNode::onScans, this, _1, _2));
  }

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan>;
  struct Detection {
    double x, y, z, width;
    int pts;
    std::vector<int> point_indices;

    // Ellipse parameters for visualization and heading estimate.
    double ellipse_yaw = 0.0;  // major-axis yaw (rad)
    double major_d = 0.3;      // diameter along major axis (m)
    double minor_d = 0.2;      // diameter along minor axis (m)
  };

  struct Track {
    int id;
    double x, y, z;
    ros::Time last_seen;

    // theta is the heading (rad) in base_frame, to be published in Pose orientation yaw.
    double theta = 0.0;

    // Store ellipse for drawing.
    double ellipse_yaw = 0.0;
    double major_d = 0.3;
    double minor_d = 0.2;
  };

  void onScans(const sensor_msgs::LaserScanConstPtr &s1,
               const sensor_msgs::LaserScanConstPtr &s2) {
    std::vector<PointXYZ> points;
    points.reserve(s1->ranges.size() + s2->ranges.size());
    if (!appendScanPoints(*s1, points)) return;
    if (!appendScanPoints(*s2, points)) return;
    if (points.empty()) return;

    std::vector<std::vector<int>> clusters;
    buildClusters(points, clusters);

    std::vector<Detection> dets;
    dets.reserve(clusters.size());
    for (const auto &cidx : clusters) {
      Detection d;
      if (!clusterToDetection(points, cidx, d)) continue;
      dets.push_back(std::move(d));
    }

    publishCandidates(s1->header.stamp, dets);
    updateTracks(s1->header.stamp, dets);
    publishTracksAndMarkers(s1->header.stamp);

    if (publish_fused_cloud_) publishFusedCloud(s1->header.stamp, points);
  }

  bool appendScanPoints(const sensor_msgs::LaserScan &scan, std::vector<PointXYZ> &out_points) {
    geometry_msgs::TransformStamped tf_msg;
    try {
      tf_msg = tf_buffer_.lookupTransform(base_frame_, scan.header.frame_id, scan.header.stamp, ros::Duration(0.1));
    } catch (const tf2::TransformException &ex) {
      ROS_WARN_THROTTLE(1.0, "TF lookup failed (%s -> %s): %s",
                        scan.header.frame_id.c_str(), base_frame_.c_str(), ex.what());
      return false;
    }

    tf2::Transform tf; tf2::fromMsg(tf_msg.transform, tf);

    const size_t n = scan.ranges.size();
    double angle = scan.angle_min;
    for (size_t i = 0; i < n; ++i, angle += scan.angle_increment) {
      const float r = scan.ranges[i];
      if (!isFiniteFloat(r)) continue;
      if (r < scan.range_min || r > scan.range_max) continue;
      if (r < range_min_ || r > range_max_) continue;

      const float xs = r * std::cos(angle);
      const float ys = r * std::sin(angle);

      tf2::Vector3 p_base = tf * tf2::Vector3(xs, ys, 0.0);
      out_points.push_back({(float)p_base.x(), (float)p_base.y(), (float)p_base.z()});
    }
    return true;
  }

  void buildClusters(const std::vector<PointXYZ> &points, std::vector<std::vector<int>> &clusters) {
    const double eps = cluster_eps_;
    const double eps2 = eps * eps;
    const double inv = 1.0 / eps;

    std::unordered_map<uint64_t, std::vector<int>> cell_map;
    cell_map.reserve(points.size() * 2);
    std::vector<std::pair<int,int>> point_cells(points.size());

    for (size_t i = 0; i < points.size(); ++i) {
      int cx = (int)std::floor(points[i].x * inv);
      int cy = (int)std::floor(points[i].y * inv);
      point_cells[i] = {cx, cy};
      cell_map[packCellKey(cx, cy)].push_back((int)i);
    }

    std::vector<uint8_t> visited(points.size(), 0);
    clusters.clear(); clusters.reserve(32);

    for (size_t i = 0; i < points.size(); ++i) {
      if (visited[i]) continue;

      std::queue<int> q;
      std::vector<int> cluster; cluster.reserve(64);

      visited[i] = 1; q.push((int)i);

      while (!q.empty()) {
        int idx = q.front(); q.pop();
        cluster.push_back(idx);

        const int cx = point_cells[idx].first;
        const int cy = point_cells[idx].second;

        for (int dx=-1; dx<=1; ++dx) for (int dy=-1; dy<=1; ++dy) {
          auto it = cell_map.find(packCellKey(cx+dx, cy+dy));
          if (it == cell_map.end()) continue;
          for (int j : it->second) {
            if (visited[j]) continue;
            const double ddx = points[j].x - points[idx].x;
            const double ddy = points[j].y - points[idx].y;
            if (ddx*ddx + ddy*ddy <= eps2) { visited[j]=1; q.push(j); }
          }
        }
      }

      if ((int)cluster.size() < cluster_min_pts_) continue;
      if ((int)cluster.size() > cluster_max_pts_) continue;
      clusters.push_back(std::move(cluster));
    }
  }

  bool clusterToDetection(const std::vector<PointXYZ> &points,
                          const std::vector<int> &cidx, Detection &out) {
    double sx=0, sy=0, sz=0;
    double minx=1e9, miny=1e9, maxx=-1e9, maxy=-1e9;

    for (int idx : cidx) {
      const auto &p = points[idx];
      sx += p.x; sy += p.y; sz += p.z;
      minx = std::min(minx, (double)p.x); miny = std::min(miny, (double)p.y);
      maxx = std::max(maxx, (double)p.x); maxy = std::max(maxy, (double)p.y);
    }

    const int n = (int)cidx.size();
    const double cx = sx / n, cy = sy / n, cz = sz / n;

    const double dx = maxx - minx;
    const double dy = maxy - miny;
    const double width = std::hypot(dx, dy);

    const double dist = std::hypot(cx, cy);
    if (dist < candidate_min_dist_ || dist > candidate_max_dist_) return false;
    if (width < cluster_min_width_ || width > cluster_max_width_) return false;

    estimateEllipseFromCluster(points, cidx, cx, cy, out.ellipse_yaw, out.major_d, out.minor_d);

    out.x=cx; out.y=cy; out.z=cz; out.width=width; out.pts=n; out.point_indices=cidx;
    return true;
  }

  void publishCandidates(const ros::Time &stamp, const std::vector<Detection> &dets) {
    geometry_msgs::PoseArray pa;
    pa.header.stamp = stamp;
    pa.header.frame_id = base_frame_;
    pa.poses.reserve(dets.size());
    for (const auto &d : dets) {
      geometry_msgs::Pose p;
      p.position.x=d.x; p.position.y=d.y; p.position.z=d.z;
      p.orientation.w=1.0;
      pa.poses.push_back(p);
    }
    pub_candidates_.publish(pa);
  }

  void updateTracks(const ros::Time &stamp, const std::vector<Detection> &dets) {
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                                 [&](const Track &t){ return (stamp - t.last_seen).toSec() > track_timeout_; }),
                  tracks_.end());

    const int T = (int)tracks_.size();
    const int D = (int)dets.size();
    std::vector<int> track_used(T,0), det_used(D,0);

    struct Pair { double dist; int ti; int di; };
    std::vector<Pair> pairs; pairs.reserve(T*D);

    for (int ti=0; ti<T; ++ti) for (int di=0; di<D; ++di) {
      double dx = dets[di].x - tracks_[ti].x;
      double dy = dets[di].y - tracks_[ti].y;
      pairs.push_back({std::hypot(dx,dy), ti, di});
    }
    std::sort(pairs.begin(), pairs.end(), [](const Pair&a, const Pair&b){ return a.dist < b.dist; });

    det_to_track_.clear();
    for (const auto &p : pairs) {
      if (p.dist > track_gate_dist_) break;
      if (track_used[p.ti] || det_used[p.di]) continue;
      tracks_[p.ti].x = dets[p.di].x;
      tracks_[p.ti].y = dets[p.di].y;
      tracks_[p.ti].z = dets[p.di].z;
      tracks_[p.ti].last_seen = stamp;
      // Update ellipse parameters.
      tracks_[p.ti].ellipse_yaw = dets[p.di].ellipse_yaw;
      tracks_[p.ti].major_d = dets[p.di].major_d;
      tracks_[p.ti].minor_d = dets[p.di].minor_d;

      // theta candidate: ellipse normal direction (major axis + 90deg).
      double th0 = normalizeAngle(dets[p.di].ellipse_yaw + M_PI_2);
      double th1 = normalizeAngle(th0 + M_PI);
      double prev = tracks_[p.ti].theta;
      double chosen = (std::fabs(angleDiff(th1, prev)) < std::fabs(angleDiff(th0, prev))) ? th1 : th0;
      tracks_[p.ti].theta = normalizeAngle(chosen + theta_offset_);

      track_used[p.ti]=1; det_used[p.di]=1;
      det_to_track_[p.di] = tracks_[p.ti].id;
    }

    for (int di=0; di<D; ++di) {
      if (det_used[di]) continue;
      Track nt;
      nt.id = next_track_id_++;
      nt.x = dets[di].x; nt.y = dets[di].y; nt.z = dets[di].z;
      nt.last_seen = stamp;
      nt.ellipse_yaw = dets[di].ellipse_yaw;
      nt.major_d = dets[di].major_d;
      nt.minor_d = dets[di].minor_d;
      nt.theta = normalizeAngle(dets[di].ellipse_yaw + M_PI_2 + theta_offset_);
      tracks_.push_back(nt);
      det_to_track_[di] = nt.id;
    }
  }

  void publishTracksAndMarkers(const ros::Time &stamp) {
    geometry_msgs::PoseArray pa;
    pa.header.stamp = stamp;
    pa.header.frame_id = base_frame_;
    pa.poses.reserve(tracks_.size());
    for (const auto &t : tracks_) {
      geometry_msgs::Pose p;
      p.position.x=t.x; p.position.y=t.y; p.position.z=t.z;
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, t.theta);
      q.normalize();
      p.orientation = tf2::toMsg(q);
      pa.poses.push_back(p);
    }
    pub_tracks_.publish(pa);

    // Publish track IDs aligned with PoseArray indices.
    std_msgs::Int32MultiArray ids;
    ids.data.reserve(tracks_.size());
    for (const auto &t : tracks_) ids.data.push_back(t.id);
    pub_track_ids_.publish(ids);

    visualization_msgs::MarkerArray ma;
    visualization_msgs::Marker del;
    del.header.stamp = stamp;
    del.header.frame_id = base_frame_;
    del.ns = "people";
    del.id = 0;
    del.action = visualization_msgs::Marker::DELETEALL;
    ma.markers.push_back(del);

    for (const auto &t : tracks_) {
      float r,g,b;
      float hue = std::fmod((t.id * 0.61803398875f), 1.0f);
      hsvToRgb(hue, 0.85f, 0.95f, r, g, b);

      visualization_msgs::Marker m;
      m.header.stamp = stamp;
      m.header.frame_id = base_frame_;
      m.ns = "people";
      m.id = t.id * 10 + 0;
      m.type = visualization_msgs::Marker::SPHERE;
      m.action = visualization_msgs::Marker::ADD;
      m.pose.position.x=t.x; m.pose.position.y=t.y; m.pose.position.z=t.z;
      m.pose.orientation.w=1.0;
      m.scale.x=m.scale.y=m.scale.z=0.25;
      m.color.r=r; m.color.g=g; m.color.b=b; m.color.a=1.0;
      m.lifetime = ros::Duration(0.2);
      ma.markers.push_back(m);

      visualization_msgs::Marker tx;
      tx.header = m.header;
      tx.ns = "people_text";
      tx.id = t.id * 10 + 1;
      tx.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      tx.action = visualization_msgs::Marker::ADD;
      tx.pose.position.x=t.x; tx.pose.position.y=t.y; tx.pose.position.z=t.z + 0.35;
      tx.pose.orientation.w=1.0;
      tx.scale.z = 0.22;
      tx.color.r=1.0; tx.color.g=1.0; tx.color.b=1.0; tx.color.a=1.0;
      tx.text = "ID:" + std::to_string(t.id);
      tx.lifetime = ros::Duration(0.2);
      ma.markers.push_back(tx);

      // Ellipse outline as a thin cylinder (scale.x/scale.y are diameters).
      visualization_msgs::Marker e;
      e.header = m.header;
      e.ns = "people_ellipse";
      e.id = t.id * 10 + 2;
      e.type = visualization_msgs::Marker::CYLINDER;
      e.action = visualization_msgs::Marker::ADD;
      e.pose.position.x = t.x; e.pose.position.y = t.y; e.pose.position.z = t.z;

      tf2::Quaternion qe;
      qe.setRPY(0.0, 0.0, t.ellipse_yaw);
      qe.normalize();
      e.pose.orientation = tf2::toMsg(qe);

      e.scale.x = t.major_d;
      e.scale.y = t.minor_d;
      e.scale.z = 0.02;

      e.color.r = r; e.color.g = g; e.color.b = b; e.color.a = 0.6;
      e.lifetime = ros::Duration(0.2);
      ma.markers.push_back(e);

    }

    pub_markers_.publish(ma);
  }

  void publishFusedCloud(const ros::Time &stamp, const std::vector<PointXYZ> &points) {
    sensor_msgs::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = base_frame_;
    cloud.height = 1;
    cloud.width = points.size();
    cloud.is_dense = false;
    cloud.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier mod(cloud);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(cloud, "z");

    for (size_t i=0; i<points.size(); ++i, ++it_x, ++it_y, ++it_z) {
      *it_x = points[i].x;
      *it_y = points[i].y;
      *it_z = points[i].z;
    }
    pub_cloud_.publish(cloud);
  }

private:
  ros::NodeHandle nh_, pnh_;
  std::string scan1_topic_, scan2_topic_, base_frame_;
  double range_min_, range_max_;
  double cluster_eps_;
  int cluster_min_pts_, cluster_max_pts_;
  double cluster_min_width_, cluster_max_width_;
  double candidate_min_dist_, candidate_max_dist_;
  double track_gate_dist_, track_timeout_;
  double theta_offset_;
  bool publish_fused_cloud_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::Publisher pub_candidates_, pub_tracks_, pub_track_ids_, pub_markers_, pub_cloud_;

  std::unique_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>> sub_scan1_, sub_scan2_;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  std::vector<Track> tracks_;
  int next_track_id_ = 1;
  std::unordered_map<int,int> det_to_track_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "lidar_people_cluster_node");
  LidarPeopleClusterNode node;
  ros::spin();
  return 0;
}