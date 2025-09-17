# 项目改进总览与代码汇编

> 生成时间：2025-09-13 08:21 (Asia/Tokyo)

本文档整理了当前项目中**已改进的要点**与**关键源码**，并附上“旧版 vs 新版”的差异对比，便于回顾与复现。
## 改善点（摘要）
- OpenCV4 适配：移除旧版 C API，统一使用 `cv::Mat / cv::` 命名空间（如粒子滤波、图像处理）。
- 追踪/阵型双模式整合：在 **objectpoint 丢失** 时仍保持**纯追踪模式**，在 **接近目标位姿** 时自动切换至 **F 阵型**。
- F 阵型进入条件可配置与放宽：例如角度阈值放宽到 `±45°`、距离上限放宽到 `~2.0m`，并加入**超时阈值**与**去重判断**抖动抑制。
- 目标朝向一致性：在阵型稳定后，增加机器人朝向与人物朝向对齐逻辑（减小最终角度误差）。
- 鲁棒的 objectpoint 获取：增加基于朝向与可见性一致性的检查（`find_objectpoint_by_orientation.cpp`），避免误检与抖动。
- 轨迹记录与可视化：`record_traj.py`/`plot_robot_person.py` 统一记录与绘制**机器人/人物**轨迹，支持异常点剔除与图例布局优化。
- ROS 集成与工具：`rosgpt.py` 辅助节点与开发调试；`person_following_multiple_OR0901.cpp` 整理多人跟随逻辑示例。

---
## 关键差异：`sks_objectpoint_robot_person`（旧 vs 新）
下方为 **Unified Diff**，可快速定位改动：

```diff
--- sks_objectpoint_robot_person_before.cpp
+++ sks_objectpoint_robot_person_new.cpp
@@ -1 +1 @@
-<!-- ERROR reading /mnt/data/sks_objectpoint_robot_person_before.cpp: [Errno 2] No such file or directory: '/mnt/data/sks_objectpoint_robot_person_before.cpp' -->+<!-- ERROR reading /mnt/data/sks_objectpoint_robot_person_new.cpp: [Errno 2] No such file or directory: '/mnt/data/sks_objectpoint_robot_person_new.cpp' -->```

## 源码：sks_objectpoint_robot_person（新）
**文件**：`sks_objectpoint_robot_person_new.cpp`

```cpp
<!-- ERROR reading /mnt/data/sks_objectpoint_robot_person_new.cpp: [Errno 2] No such file or directory: '/mnt/data/sks_objectpoint_robot_person_new.cpp' -->
```
## 源码：sks_objectpoint_robot_person（旧）
**文件**：`sks_objectpoint_robot_person_before.cpp`

```cpp
<!-- ERROR reading /mnt/data/sks_objectpoint_robot_person_before.cpp: [Errno 2] No such file or directory: '/mnt/data/sks_objectpoint_robot_person_before.cpp' -->
```
## 源码：hdk_objectpoint_finder（objectpoint 检测）
**文件**：`hdk_objectpoint_finder.cpp`

```cpp
<!-- ERROR reading /mnt/data/hdk_objectpoint_finder.cpp: [Errno 2] No such file or directory: '/mnt/data/hdk_objectpoint_finder.cpp' -->
```
## 源码：find_objectpoint_by_orientation（朝向一致性辅助）
**文件**：`find_objectpoint_by_orientation.cpp`

```cpp
<!-- ERROR reading /mnt/data/find_objectpoint_by_orientation.cpp: [Errno 2] No such file or directory: '/mnt/data/find_objectpoint_by_orientation.cpp' -->
```
## 源码：person_following_multiple_OR0901（多人跟随示例）
**文件**：`person_following_multiple_OR0901.cpp`

```cpp
<!-- ERROR reading /mnt/data/person_following_multiple_OR0901.cpp: [Errno 2] No such file or directory: '/mnt/data/person_following_multiple_OR0901.cpp' -->
```
## 源码：record_traj（轨迹记录）
**文件**：`record_traj.py`

```python
<!-- ERROR reading /mnt/data/record_traj.py: [Errno 2] No such file or directory: '/mnt/data/record_traj.py' -->
```
## 源码：plot_robot_person（轨迹可视化）
**文件**：`plot_robot_person.py`

```python
<!-- ERROR reading /mnt/data/plot_robot_person.py: [Errno 2] No such file or directory: '/mnt/data/plot_robot_person.py' -->
```
## 源码：rosgpt（辅助/工具节点）
**文件**：`rosgpt.py`

```python
<!-- ERROR reading /mnt/data/rosgpt.py: [Errno 2] No such file or directory: '/mnt/data/rosgpt.py' -->
```

---

## 使用与建议
- **编译**：确保 `CMakeLists.txt` 与 `package.xml` 已适配 OpenCV4 与依赖的 ROS 包（`cv_bridge`, `geometry_msgs`, `sensor_msgs` 等）。
- **运行**：
  - 纯追踪模式：启动基础感知与跟随节点，未获取 objectpoint 时仍持续追踪。
  - 阵型模式：当接近目标位姿（距离/角度阈值满足）自动切换，进入 F 阵型；加入超时与去重判定以防止频繁切换。
- **可视化**：运行 `record_traj.py` 保存 CSV，再用 `plot_robot_person.py` 统一绘制机器人/人物轨迹；必要时在绘制侧做异常点剔除。
- **参数调优**：角度阈值（如 `±45°`）、距离阈值（如 `2.0m`）、超时阈值（如 `3s`）应根据实验环境与传感器噪声进行细化。
