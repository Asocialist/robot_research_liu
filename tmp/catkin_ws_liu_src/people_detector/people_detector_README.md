# people_detector

LiDAR 多人检测器（ROS1/Noetic）。  
从 2D 激光雷达 `/scan` 做**几何聚类**，筛出“人体大小”的簇，输出所有疑似人的位置（质心），并提供 RViz 可视化。  
不依赖反射服，适合作为你现有 `person_follow`（追踪器）的**上游感知**。

## ✨ 特点

- 仅用 2D LiDAR，**无相机/反射服依赖**  
- 扫描序**邻接聚类** + **人体几何门限**（点数 / 直径）  
- 发布 `PoseArray` 和 `MarkerArray`，开箱即用 RViz  
- 结构简单、参数可调、易与下游融合（F-formation / destination_queue）

---

## 🚦 接口（ROS Topics）

**订阅**
- `/scan` (`sensor_msgs/LaserScan`)

**发布**
- `/people/poses` (`geometry_msgs/PoseArray`)：每帧所有“人”的位姿（位置 = 簇质心；朝向暂置单位四元数）
- `/people/markers` (`visualization_msgs/MarkerArray`)：RViz 圆柱可视化
- `/people/clusters` (`sensor_msgs/PointCloud`)（可选）：调参/诊断用的聚类点云

---

## ⚙️ 主要参数（rosparam，命名空间 `~`）

| 参数名 | 类型 | 默认 | 说明 |
|---|---|---:|---|
| `min_range` | double | 0.1 | 接收点的最小量程（米） |
| `max_range` | double | 10.0 | 接收点的最大量程（米） |
| `downsample_step` | int | 1 | 扫描降采样步长（1=不过采） |
| `cluster_dist_base` | double | 0.10 | 相邻点聚类的基础阈值（米） |
| `cluster_dist_scale` | double | 0.04 | 距离自适应系数，阈值 = `max(base, r*scale)` |
| `min_cluster_points` | int | 3 | 最小簇点数 |
| `max_cluster_points` | int | 80 | 最大簇点数 |
| `person_min_diameter` | double | 0.20 | 人体直径下限（米） |
| `person_max_diameter` | double | 0.80 | 人体直径上限（米） |
| `marker_lifetime` | double | 0.2 | RViz 标记生存时间（秒） |
| `frame_id` | string | "" | 为空则使用 `/scan` 的 frame |

---

## 🧩 代码结构（关键文件）

```
people_detector/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── detector.launch         # 示例启动
└── src/
    └── people_detector_lidar.cpp
```

---

## 🧠 算法流程（概览）

1. **极坐标 → 笛卡尔**：将 `(range, angle)` 转为 `(x, y)`  
2. **扫描序邻接聚类**：相邻点距离小于自适应阈值 → 归为同一簇  
   - 阈值：`th = max(cluster_dist_base, range * cluster_dist_scale)`  
3. **人体筛选**：对每个簇计算  
   - 点数 ∈ `[min_cluster_points, max_cluster_points]`  
   - 空间直径（最大点间距离）∈ `[person_min_diameter, person_max_diameter]`  
4. **输出**：对通过的簇取质心 → `PoseArray`；同时发 RViz `MarkerArray`

---

## 🧩 函数与类（people_detector_lidar.cpp）

> 注：源码内注释为英文；此处为中文摘要说明。

### 结构体 `Params`
- 保存所有运行参数（见“主要参数”表）

### 类 `PeopleDetectorLidar`
- 节点主体：订阅 `/scan`，完成聚类与发布

**公开：**
- `PeopleDetectorLidar(ros::NodeHandle& nh, ros::NodeHandle& pnh)`  
  读取参数、注册订阅/发布，打印启动日志。

**私有：**
- `void scanCb(const sensor_msgs::LaserScanConstPtr& scan)`  
  **回调入口**。执行完整处理：聚类 → 筛选 → 生成 `PoseArray` 与 `MarkerArray` → 发布。  
  若订阅者存在，额外发布 `/people/clusters` 调参点云。

- `void formClusters(const LaserScanConstPtr& scan, std::vector<std::vector<Point32>>& clusters, sensor_msgs::PointCloud* all_pts_opt)`  
  **扫描序邻接聚类**。逐点判断与前一有效点的距离是否小于阈值；按序分段组成簇。可收集全体点用于调参可视化。

- `bool isPersonLike(const std::vector<Point32>& cluster, double& out_diameter) const`  
  **人体筛选**。检查点数范围；计算簇内最大两点距离作为直径，并判定是否落在人体直径区间。返回布尔和直径值。

- `static geometry_msgs::Point32 centroid(const std::vector<Point32>& cluster)`  
  计算簇的**质心**（平均值）。

- `visualization_msgs::Marker makeMarker(const Point32& c, int id, const std::string& ns, const std::string& frame, double lifetime) const`  
  构造 RViz **圆柱**标记（位置在质心，统一大小/颜色/寿命）。

- `static inline double sqrDist(const Point32& a, const Point32& b)`  
  **工具**：平方距离，避免频繁 `sqrt`。

- `static inline bool polarToXY(double r, double angle, Point32& out)`  
  **工具**：极坐标转平面坐标，过滤无效距离。

### `int main(int argc, char** argv)`
- 初始化 ROS、构建 `PeopleDetectorLidar`、`ros::spin()` 进入回调循环。

---

## ▶️ 运行

### 编译
```bash
cd ~/catkin_ws_liu
catkin build   # 或 catkin_make
source devel/setup.bash
```

### 启动（示例）
```bash
roslaunch people_detector detector.launch
```

**RViz**：  
- Fixed Frame 设为雷达 frame（如 `base_link`/`laser`）  
- 添加 `PoseArray`（/people/poses）与 `MarkerArray`（/people/markers）

---

## 🔧 调参建议（最小可验证顺序）

1. **空场**：确保无误检  
   - 噪声多 → 提高 `min_cluster_points` / `cluster_dist_base`  
2. **单人站立/移动**：  
   - 断裂 → 增大 `cluster_dist_scale`  
   - 被并入背景 → 减小 `cluster_dist_scale` 或 `max_cluster_points`  
3. **多人靠近**：  
   - 合并成大簇 → 减小 `cluster_dist_scale`，必要时降低 `person_max_diameter`

---

## 🤝 与现有系统的衔接

- **person_follow（追踪器）**：保持只跟“反射服”目标；  
- **people_detector（检测器）**：输出所有人；  
- 在你的 UI/融合层订阅 `/people/poses`，可在 `person_follow` 的 OpenCV 窗口**叠加绿圈+箭头**并在未跟踪时**用检测播种**（你已完成整合版）。

---

## 🧪 常见问题

- **看不到标记**：  
  - RViz 的 Fixed Frame 与发布 frame 不一致 → 通过 `~frame_id` 参数强制或修正 TF  
- **始终 0 人**：  
  - 检查 `/scan` 数据是否合理；减少 `person_min_diameter`（例如 0.15）做探底；  
  - 放宽 `cluster_dist_scale`（远距离点更稀疏）。  
- **多人被并到一起**：  
  - 降低 `cluster_dist_scale`；降低 `max_cluster_points`；  
  - 后续可加入“腿部双峰分裂”改进（待办）。

---

## 📜 许可证

BSD (与包内源文件头一致)

---

## 🗺️ 未来路线（可选增量）

- 质心**速度估计**（EMA/卡尔曼），便于 ID 关联  
- **多目标关联**（Hungarian / ByteTrack）形成稳定 track 列表  
- 自定义消息：`DetectedPerson{id, pose, velocity, confidence}`  
- 与 `nkm_destination_queue` 的策略桥接（检测→队列→sbtp）
