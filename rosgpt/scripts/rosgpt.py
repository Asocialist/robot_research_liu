#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
rosgpt.py (chat-only, pose-aware, dual-source robot pose)

Purpose:
- Chat with the user using live context (robot & person poses) WITHOUT deciding goals or waypoints.
- Publishes plain text replies only to /rosgptpublish.
- Supports two robot-pose sources via param (~robot_pose_type):
    "pose2d": geometry_msgs/Pose2D (e.g., /traj/robot_map)
    "amcl"  : geometry_msgs/PoseWithCovarianceStamped (e.g., /amcl_pose)

Params (ROS):
- ~robot_pose_type  (default: "pose2d")        : "pose2d" | "amcl"
- ~robot_pose_topic (default: "/traj/robot_map"): topic for robot pose
- ~person_pose_topic(default: "/traj/person_map"): topic for person pose (Pose2D)
- ~chat_only        (default: true)             : keep goal features disabled
- ~model_name       (default: "gpt-3.5-turbo")  : LLM model name
- ~wp_label_file    (default: "")               : kept for forward-compatibility (unused here)

Topics (Subscribe):
- /rosgptinput          : std_msgs/String (user text)
- <robot_pose_topic>    : Pose2D OR PoseWithCovarianceStamped (per ~robot_pose_type)
- <person_pose_topic>   : Pose2D (map frame)

Topics (Publish):
- /rosgptpublish        : std_msgs/String   (chat text reply)
- /gpt_plan/debug       : std_msgs/String   (JSON context for debugging)
- /gpt_next_wp          : std_msgs/String   (EXISTS but UNUSED in chat-only mode)
- /gpt_goal_pose        : geometry_msgs/PoseStamped (EXISTS but UNUSED in chat-only mode)
- /gpt_plan/waypoints   : nav_msgs/Path     (EXISTS but UNUSED in chat-only mode)

Env:
- OPENAI_API_KEY optional; if missing, node falls back to local echo replies.

Note:
- All goal-picking & path-publishing are DISABLED on purpose to avoid interfering with navigation.
"""

import os
import json
import math
import traceback

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose2D, PoseWithCovarianceStamped
from nav_msgs.msg import Path

# Try to import OpenAI SDK; allow running without it
try:
    import openai
    _HAS_OPENAI = True
except Exception:
    openai = None
    _HAS_OPENAI = False


def quat_to_yaw(q):
    """Convert quaternion (geometry_msgs/Quaternion) to yaw (rad)."""
    # yaw-only extraction; assumes normalized quaternion
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def fmt_pose(p):
    """Compact pretty-print for pose dicts."""
    if not isinstance(p, dict):
        return "unknown"
    def _f(x):
        try:
            return float(x)
        except Exception:
            return x
    x = _f(p.get("x", None))
    y = _f(p.get("y", None))
    yaw = _f(p.get("yaw", p.get("theta", None)))
    if yaw is None:
        return f"(x={x:.3f}, y={y:.3f})" if isinstance(x, float) and isinstance(y, float) else str(p)
    try:
        return f"(x={x:.3f}, y={y:.3f}, yaw={yaw:.3f})"
    except Exception:
        return str(p)


class ROSGPTNode(object):
    def __init__(self):
        # -------------------------
        # ROS init & configuration
        # -------------------------
        rospy.init_node("rosgpt_node")

        # Parameters
        self.robot_pose_type  = rospy.get_param("~robot_pose_type", "pose2d")  # "pose2d" | "amcl"
        self.robot_pose_topic = rospy.get_param("~robot_pose_topic", "/traj/robot_map")
        self.person_pose_topic= rospy.get_param("~person_pose_topic", "/traj/person_map")

        self.chat_only        = rospy.get_param("~chat_only", True)  # MUST be True in this version
        self.model_name       = rospy.get_param("~model_name", "gpt-3.5-turbo")
        self.wp_label_file    = rospy.get_param("~wp_label_file", "")  # kept for compatibility

        # OpenAI API key
        self.api_key = os.getenv("OPENAI_API_KEY", "")
        if not self.api_key or not _HAS_OPENAI:
            rospy.logwarn("OpenAI disabled (no SDK or no OPENAI_API_KEY). Running in local fallback mode.")
        else:
            openai.api_key = self.api_key

        # Cached states
        self.robot_pose  = None  # {"x": float, "y": float, "yaw": float, "stamp": float}
        self.person_pose = None  # {"x": float, "y": float, "theta": float, "stamp": float}
        self.chat_history = []

        # Publishers (keep extra pubs for future but DO NOT use them in chat-only mode)
        self.pub_reply = rospy.Publisher("/rosgptpublish", String, queue_size=10)
        self.pub_next_wp   = rospy.Publisher("/gpt_next_wp", String, queue_size=1)
        self.pub_goal_pose = rospy.Publisher("/gpt_goal_pose", PoseStamped, queue_size=1)
        self.pub_path_vis  = rospy.Publisher("/gpt_plan/waypoints", Path, queue_size=1)
        self.pub_dbg       = rospy.Publisher("/gpt_plan/debug", String, queue_size=10)

        # Subscribers
        rospy.Subscriber("/rosgptinput", String, self.cb_user_text, queue_size=1)

        # Robot pose subscriber (dual-source switch)
        if self.robot_pose_type.lower() == "amcl":
            rospy.Subscriber(self.robot_pose_topic, PoseWithCovarianceStamped,
                             self.cb_robot_pose_amcl, queue_size=1)
            rospy.loginfo("rosgpt_node: robot pose source = AMCL (%s)", self.robot_pose_topic)
        else:
            rospy.Subscriber(self.robot_pose_topic, Pose2D,
                             self.cb_robot_pose2d, queue_size=1)
            rospy.loginfo("rosgpt_node: robot pose source = Pose2D (%s)", self.robot_pose_topic)

        # Person pose subscriber (Pose2D in map frame)
        rospy.Subscriber(self.person_pose_topic, Pose2D, self.cb_person_pose2d, queue_size=1)

        rospy.loginfo("rosgpt_node (chat-only) ready. person_pose_topic=%s", self.person_pose_topic)

    # -------------------------
    # Pose callbacks
    # -------------------------
    def cb_robot_pose_amcl(self, msg: PoseWithCovarianceStamped):
        """Cache robot pose from AMCL (map frame)."""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.robot_pose = {
            "x": float(p.x),
            "y": float(p.y),
            "yaw": float(quat_to_yaw(q)),
            "stamp": rospy.Time.now().to_sec()
        }

    def cb_robot_pose2d(self, msg: Pose2D):
        """Cache robot pose from Pose2D (already in map frame)."""
        self.robot_pose = {
            "x": float(msg.x),
            "y": float(msg.y),
            "yaw": float(msg.theta),
            "stamp": rospy.Time.now().to_sec()
        }

    def cb_person_pose2d(self, msg: Pose2D):
        """Cache person pose (Pose2D in map frame)."""
        self.person_pose = {
            "x": float(msg.x),
            "y": float(msg.y),
            "theta": float(msg.theta),
            "stamp": rospy.Time.now().to_sec()
        }

    # -------------------------
    # User text handling
    # -------------------------
    def cb_user_text(self, msg: String):
        """Receive user text -> produce chat reply using live context. NO goal decision here."""
        user_msg = (msg.data or "").strip()
        if not user_msg:
            rospy.logwarn("Empty /rosgptinput received.")
            return

        # Compose light context snapshot
        ctx = {
            "robot_pose": self.robot_pose if self.robot_pose else "unknown",
            "person_pose": self.person_pose if self.person_pose else "unknown"
        }

        # Emit debug JSON for auditing
        try:
            self.pub_dbg.publish(json.dumps({"ctx": ctx, "user": user_msg}, ensure_ascii=False))
        except Exception:
            self.pub_dbg.publish(String(data="(debug serialize error)"))

        # Chat reply
        reply = self._chat_reply_with_context(user_msg, ctx)
        if reply:
            self.pub_reply.publish(reply)

        # IMPORTANT: In chat-only mode we do NOT pick or publish any waypoint/goal.
        # self._pick_next_waypoint_label(...)  # <-- intentionally disabled

    # -------------------------
    # GPT chat (context-aware, goal-free)
    # -------------------------
    def _chat_reply_with_context(self, user_msg: str, ctx: dict) -> str:
        """
        Use LLM (or fallback) to answer with awareness of robot/person poses.
        MUST NOT decide or suggest any waypoint/goal or navigation command.
        """
        # Append to short memory
        self.chat_history.append({"role": "user", "content": user_msg})
        self.chat_history = self.chat_history[-8:]

        # Fallback if no OpenAI
        if not _HAS_OPENAI or not self.api_key:
            r = fmt_pose(self.robot_pose)
            p = fmt_pose(self.person_pose)
            return f"（local model）recieved：{user_msg}\n status：robot{r}；person{p}"

        try:
            sys_prompt = (
                "You are a robot shopping assistant. "
                "You are given a live context JSON containing robot and person poses. "
                "Explain the situation, answer user questions, and maintain natural conversation. "
                "VERY IMPORTANT: Do NOT decide, suggest, or output any navigation goals, "
                "waypoints, or commands. Keep responses concise and helpful."
            )
            content_ctx = json.dumps({"context": ctx}, ensure_ascii=False)

            resp = openai.ChatCompletion.create(
                model=self.model_name,
                messages=[
                    {"role": "system", "content": sys_prompt},
                    {"role": "user", "content": content_ctx},
                    {"role": "user", "content": user_msg},
                ],
                temperature=0.2,
                max_tokens=256,
            )
            text = resp.choices[0].message["content"].strip()
            self.chat_history.append({"role": "assistant", "content": text})
            self.chat_history = self.chat_history[-8:]
            return text
        except Exception as e:
            rospy.logwarn("OpenAI chat failed: %s", str(e))
            rospy.logdebug(traceback.format_exc())
            r = fmt_pose(self.robot_pose)
            p = fmt_pose(self.person_pose)
            return f"（local model）error。status：robot{r}；person{p}"


if __name__ == "__main__":
    node = ROSGPTNode()
    rospy.loginfo("rosgpt_node (chat-only) running. Waiting for /rosgptinput ...")
    rospy.spin()
