// ===== opencv_viewer_node.cpp (full) =====
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Quaternion.h>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <vector>
#include <string>
#include <cmath>

static inline double yawFromQuat(const geometry_msgs::Quaternion &q) {
  // Standard yaw extraction from quaternion.
  const double siny_cosp = 2.0 * (q.w*q.z + q.x*q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}


struct Pt2 { float x; float y; };

class OpenCVViewerNode {
public:
  OpenCVViewerNode() : nh_(), pnh_("~") {
    pnh_.param<std::string>("cloud_topic", cloud_topic_, "/scan_fused_cloud");
    pnh_.param<std::string>("marker_topic", marker_topic_, "/people_markers");
    pnh_.param<int>("image_size", image_size_, 800);
    pnh_.param<double>("pixels_per_meter", ppm_, 150.0);
    pnh_.param<double>("draw_range_m", draw_range_m_, 6.0);
    pnh_.param<int>("point_radius", point_radius_, 1);
    pnh_.param<int>("person_radius", person_radius_, 6);
    pnh_.param<bool>("draw_axes", draw_axes_, true);

    sub_cloud_ = nh_.subscribe(cloud_topic_, 1, &OpenCVViewerNode::onCloud, this);
    sub_markers_ = nh_.subscribe(marker_topic_, 1, &OpenCVViewerNode::onMarkers, this);

    window_name_ = "lidar_top_view";
    cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
    cv::resizeWindow(window_name_, image_size_, image_size_);
  }

  void spin() {
    ros::Rate rate(30);
    while (ros::ok()) {
      ros::spinOnce();
      renderOnce();
      rate.sleep();
    }
  }

private:
  void onCloud(const sensor_msgs::PointCloud2ConstPtr &msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    points_.clear();
    points_.reserve(msg->width * msg->height);

    sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");
    const double r2 = draw_range_m_ * draw_range_m_;

    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
      float x = *it_x, y = *it_y;
      (void)*it_z;
      if (!std::isfinite(x) || !std::isfinite(y)) continue;
      if (x*x + y*y > r2) continue;
      points_.push_back({x,y});
    }
  }

  void onMarkers(const visualization_msgs::MarkerArrayConstPtr &msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    markers_ = *msg;
  }

  cv::Point worldToPixel(double x, double y) const {
    const double cx = image_size_ * 0.5;
    const double cy = image_size_ * 0.5;
    int u = (int)std::lround(cx + x * ppm_);
    int v = (int)std::lround(cy - y * ppm_);
    return cv::Point(u,v);
  }

  void drawAxes(cv::Mat &img) const {
    const cv::Point c(image_size_/2, image_size_/2);
    cv::circle(img, c, 4, cv::Scalar(200,200,200), -1);
    cv::arrowedLine(img, c, worldToPixel(1.0,0.0), cv::Scalar(0,0,255), 2, cv::LINE_AA);
    cv::arrowedLine(img, c, worldToPixel(0.0,1.0), cv::Scalar(0,255,0), 2, cv::LINE_AA);
  }

  void renderOnce() {
    std::vector<Pt2> pts;
    visualization_msgs::MarkerArray mk;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      pts = points_;
      mk = markers_;
    }

    cv::Mat img(image_size_, image_size_, CV_8UC3, cv::Scalar(0,0,0));
    if (draw_axes_) drawAxes(img);

    for (const auto &p : pts) {
      auto uv = worldToPixel(p.x, p.y);
      if (uv.x<0||uv.x>=image_size_||uv.y<0||uv.y>=image_size_) continue;
      cv::circle(img, uv, point_radius_, cv::Scalar(180,180,180), -1);
    }

    for (const auto &m : mk.markers) {
      if (m.action == visualization_msgs::Marker::DELETEALL) continue;

      // 1) Draw ellipse markers (people_ellipse, CYLINDER).
      if (m.ns == "people_ellipse" && m.type == visualization_msgs::Marker::CYLINDER) {
        auto uv = worldToPixel(m.pose.position.x, m.pose.position.y);
        if (uv.x<0||uv.x>=image_size_||uv.y<0||uv.y>=image_size_) continue;

        int b = (int)std::lround(m.color.b * 255.0);
        int g = (int)std::lround(m.color.g * 255.0);
        int r = (int)std::lround(m.color.r * 255.0);

        // scale.x/scale.y are diameters in meters.
        const double a_px = 0.5 * m.scale.x * ppm_;
        const double b_px = 0.5 * m.scale.y * ppm_;
        if (a_px < 1.0 || b_px < 1.0) continue;

        // Convert world yaw to image rotation (y is flipped in worldToPixel).
        const double yaw = yawFromQuat(m.pose.orientation);
        const double angle_deg = -yaw * 180.0 / M_PI;

        cv::ellipse(img, uv,
                    cv::Size((int)std::lround(a_px), (int)std::lround(b_px)),
                    angle_deg, 0, 360,
                    cv::Scalar(b,g,r), 2, cv::LINE_AA);
        continue;
      }

      // 2) Draw person centers (people, SPHERE).
      if (m.ns == "people" && m.type == visualization_msgs::Marker::SPHERE) {
        auto uv = worldToPixel(m.pose.position.x, m.pose.position.y);
        if (uv.x<0||uv.x>=image_size_||uv.y<0||uv.y>=image_size_) continue;

        int b = (int)std::lround(m.color.b * 255.0);
        int g = (int)std::lround(m.color.g * 255.0);
        int r = (int)std::lround(m.color.r * 255.0);

        cv::circle(img, uv, person_radius_, cv::Scalar(b,g,r), -1, cv::LINE_AA);
        cv::circle(img, uv, person_radius_+2, cv::Scalar(255,255,255), 1, cv::LINE_AA);

        int track_id = m.id / 10;
        cv::putText(img, "ID:"+std::to_string(track_id), uv+cv::Point(8,-8),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255), 1, cv::LINE_AA);
        continue;
      }
    }

cv::imshow(window_name_, img);
    int key = cv::waitKey(1);
    if (key=='q' || key==27) ros::shutdown();
  }

private:
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_cloud_, sub_markers_;
  std::string cloud_topic_, marker_topic_;
  int image_size_;
  double ppm_, draw_range_m_;
  int point_radius_, person_radius_;
  bool draw_axes_;
  std::string window_name_;
  std::mutex mtx_;
  std::vector<Pt2> points_;
  visualization_msgs::MarkerArray markers_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "opencv_viewer_node");
  OpenCVViewerNode node;
  node.spin();
  return 0;
}