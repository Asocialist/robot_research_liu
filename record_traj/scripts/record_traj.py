#!/usr/bin/env python3
# coding: utf-8
import math
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
    关键特性：
      - 机器人姿态最近邻匹配（基于到达时间）
      - 机器人姿态时效性检查（超过阈值则丢弃该帧或按参数允许）
    """
    def __init__(self):
        # 参数：
        self.is_tracking_global = rospy.get_param("~is_tracking_global", False)  # /pose_person_following 是否已是 map 系
        self.buffer_len         = rospy.get_param("~buffer_len", 200)            # 机器人姿态缓冲长度
        self.max_dt_sec         = rospy.get_param("~max_dt_sec", 0.20)           # 最近邻可接受的最大时间差（秒）
        self.allow_stale        = rospy.get_param("~allow_stale", False)         # 超阈值是否仍然使用（默认丢弃）

        # 发布器
        self.pub_robot  = rospy.Publisher("/traj/robot_map",  Pose2D, queue_size=100)
        self.pub_person = rospy.Publisher("/traj/person_map", Pose2D, queue_size=100)

        # 订阅器
        rospy.Subscriber("/pose_particle_localizer", msg_pose2d_stamped, self.robot_cb,  queue_size=200)
        rospy.Subscriber("/pose_person_following",  Pose2D,              self.person_cb, queue_size=200)

        # 机器人姿态缓冲：元素为 (stamp_sec, x, y, theta)
        self.robot_buf = deque(maxlen=int(self.buffer_len))

        rospy.loginfo("[traj_standardizer_nn] params: is_tracking_global=%s, buffer_len=%d, max_dt_sec=%.3f, allow_stale=%s",
                      self.is_tracking_global, self.buffer_len, self.max_dt_sec, self.allow_stale)

    # 记录机器人位姿：用到达时间戳
    def robot_cb(self, msg):
        now = rospy.Time.now().to_sec()
        self.robot_buf.append((now, msg.x, msg.y, msg.theta))
        # 直接把机器人位姿发布为 /traj/robot_map（它本来就是 map 系）
        self.pub_robot.publish(Pose2D(x=msg.x, y=msg.y, theta=msg.theta))

    # 人物到达：做最近邻匹配 + 时效性检查，再做局部->map 变换（若需要）
    def person_cb(self, p):
        # 若人物本身就是 map 系，直接发布（无需匹配/变换）
        if self.is_tracking_global:
            self.pub_person.publish(Pose2D(x=p.x, y=p.y, theta=normalize_angle(p.theta)))
            return

        # 否则：人物是“相对机器人”的局部系，需要最近邻机器人姿态
        if not self.robot_buf:
            rospy.logwarn_throttle(2.0, "[traj_standardizer_nn] robot pose buffer empty; skip person frame")
            return

        t_now = rospy.Time.now().to_sec()
        # 最近邻匹配（按到达时间）
        tt, xr, yr, tr = min(self.robot_buf, key=lambda z: abs(z[0] - t_now))
        dt = abs(t_now - tt)

        if dt > self.max_dt_sec:
            rospy.logwarn_throttle(1.0,
                "[traj_standardizer_nn] stale robot pose: |dt|=%.3fs > %.3fs (allow_stale=%s)",
                dt, self.max_dt_sec, self.allow_stale)
            if not self.allow_stale:
                return  # 丢弃该帧，保证严谨

        # 用匹配到的机器人姿态做 2D 刚体变换：base_link -> map
        c, s = math.cos(tr), math.sin(tr)
        X = xr + c*p.x - s*p.y
        Y = yr + s*p.x + c*p.y
        T = normalize_angle(tr + p.theta)

        self.pub_person.publish(Pose2D(x=X, y=Y, theta=T))

if __name__ == "__main__":
    rospy.init_node("traj_standardizer_nn")
    node = TrajStdNN()
    rospy.loginfo("traj_standardizer_nn publishing /traj/robot_map and /traj/person_map (both in map frame)")
    rospy.spin()
