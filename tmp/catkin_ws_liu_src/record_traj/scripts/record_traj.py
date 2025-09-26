#!/usr/bin/env python3
# coding: utf-8
import math, time
from collections import deque
import rospy
from geometry_msgs.msg import Pose2D
from gnd_msgs.msg import msg_pose2d_stamped

def normalize_angle(a):
    while a >  math.pi: a -= 2.0*math.pi
    while a < -math.pi: a += 2.0*math.pi
    return a

class TrajStdNN:
    """
    记录/标准化轨迹（map系）：
      - /traj/robot_map : Pose2D (map)
      - /traj/person_map: Pose2D (map)
    特性：
      - 机器人姿态最近邻匹配（到达时间）
      - 机器人姿态时效性检查
      - NEW: 空间异常点剔除（位置跳变 & 速度阈值），分别作用于 robot 与 person
    """
    def __init__(self):
        # —— 参数 ——
        self.is_tracking_global = rospy.get_param("~is_tracking_global", False)  # /pose_person_following 是否已是 map 系
        self.buffer_len         = rospy.get_param("~buffer_len", 200)            # 机器人姿态缓冲长度
        self.max_dt_sec         = rospy.get_param("~max_dt_sec", 0.20)           # 最近邻可接受的最大时间差（秒）
        self.allow_stale        = rospy.get_param("~allow_stale", False)         # 超阈值是否仍然使用（默认丢弃）

        # NEW: 空间异常点阈值（可按场景调整）
        self.max_jump_m_robot   = rospy.get_param("~max_jump_m_robot",   1.5)
        self.max_speed_mps_robot= rospy.get_param("~max_speed_mps_robot",3.0)
        self.max_jump_m_person  = rospy.get_param("~max_jump_m_person",  2.0)
        self.max_speed_mps_person=rospy.get_param("~max_speed_mps_person",3.0)

        # 发布器
        self.pub_robot  = rospy.Publisher("/traj/robot_map",  Pose2D, queue_size=100)
        self.pub_person = rospy.Publisher("/traj/person_map", Pose2D, queue_size=100)

        # 订阅器
        rospy.Subscriber("/pose_particle_localizer", msg_pose2d_stamped, self.robot_cb,  queue_size=200)
        rospy.Subscriber("/pose_person_following",  Pose2D,              self.person_cb, queue_size=200)

        # 机器人姿态缓冲：元素为 (stamp_sec, x, y, theta)
        self.robot_buf = deque(maxlen=int(self.buffer_len))

        # NEW: 最近一次成功发布到 /traj/* 的点（用于空间异常检测）
        self.prev_robot  = None  # dict: {"t":sec, "x":m, "y":m, "th":rad}
        self.prev_person = None

        rospy.loginfo(
            "[traj_standardizer_nn] is_tracking_global=%s, buffer_len=%d, max_dt_sec=%.3f, allow_stale=%s; "
            "robot(jump=%.2f,speed=%.2f) person(jump=%.2f,speed=%.2f)",
            self.is_tracking_global, self.buffer_len, self.max_dt_sec, self.allow_stale,
            self.max_jump_m_robot, self.max_speed_mps_robot, self.max_jump_m_person, self.max_speed_mps_person
        )

    # —— 工具：空间异常判断（对上一帧成功发布的点做增量判定） ——
    def _is_spatial_outlier(self, prev, t, x, y, jump_th, speed_th):
        if prev is None:
            return False
        dt = max(t - prev["t"], 1e-6)
        dx, dy = x - prev["x"], y - prev["y"]
        dist   = (dx*dx + dy*dy) ** 0.5
        speed  = dist / dt
        if dist > jump_th or speed > speed_th:
            return True
        return False

    # —— 机器人回调：记录 + 发布（带空间异常过滤） ——
    def robot_cb(self, msg):
        now = rospy.Time.now().to_sec()
        xr, yr, tr = msg.x, msg.y, msg.theta

        # 空间异常点检测（相对于“上次成功发布的机器人点”）
        if self._is_spatial_outlier(self.prev_robot, now, xr, yr,
                                    self.max_jump_m_robot, self.max_speed_mps_robot):
            rospy.logwarn_throttle(1.0,
                "[traj_standardizer_nn] drop robot outlier: jump/speed over limit")
            # 仍把该帧加入缓冲（用于 NN 匹配），但不发布到 /traj/robot_map
            self.robot_buf.append((now, xr, yr, tr))
            return

        # 正常：写入缓冲并发布
        self.robot_buf.append((now, xr, yr, tr))
        self.pub_robot.publish(Pose2D(x=xr, y=yr, theta=tr))
        self.prev_robot = {"t": now, "x": xr, "y": yr, "th": tr}

    # —— 人物回调：最近邻 + 时效性检查 + 变换 + 空间异常过滤 ——
    def person_cb(self, p):
        t_now = rospy.Time.now().to_sec()

        # 若人物本身就是 map 系，直接使用
        if self.is_tracking_global:
            X, Y, T = p.x, p.y, normalize_angle(p.theta)
        else:
            # 需要最近邻机器人姿态
            if not self.robot_buf:
                rospy.logwarn_throttle(2.0,
                    "[traj_standardizer_nn] robot pose buffer empty; skip person")
                return

            # 最近邻匹配（按到达时间）
            tt, xr, yr, tr = min(self.robot_buf, key=lambda z: abs(z[0] - t_now))
            dt = abs(t_now - tt)
            if dt > self.max_dt_sec and not self.allow_stale:
                rospy.logwarn_throttle(1.0,
                    "[traj_standardizer_nn] stale robot pose: |dt|=%.3fs > %.3fs -> drop person",
                    dt, self.max_dt_sec)
                return

            # base_link -> map 变换
            c, s = math.cos(tr), math.sin(tr)
            X = xr + c*p.x - s*p.y
            Y = yr + s*p.x + c*p.y
            T = normalize_angle(tr + p.theta)

        # NEW: 人物空间异常检测（相对于“上次成功发布的人物点”）
        if self._is_spatial_outlier(self.prev_person, t_now, X, Y,
                                    self.max_jump_m_person, self.max_speed_mps_person):
            rospy.logwarn_throttle(1.0,
                "[traj_standardizer_nn] drop person outlier: jump/speed over limit")
            return

        # 发布
        self.pub_person.publish(Pose2D(x=X, y=Y, theta=T))
        self.prev_person = {"t": t_now, "x": X, "y": Y, "th": T}

if __name__ == "__main__":
    rospy.init_node("traj_standardizer_nn")
    node = TrajStdNN()
    rospy.loginfo("traj_standardizer_nn -> /traj/robot_map & /traj/person_map (map frame, with spatial outlier filter)")
    rospy.spin()
