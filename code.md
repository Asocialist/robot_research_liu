# openai_bot/ROS package template

## CMakeLists.txt
cmake_minimum_required(VERSION 3.0.2)
project(openai_bot)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/chat_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## package.xml
<?xml version="1.0"?>
<package format="2">
  <name>openai_bot</name>
  <version>0.0.1</version>
  <description>ROS node to interface with OpenAI API</description>
  <maintainer email="your@email.com">Your Name</maintainer>
  <license>MIT</license>
  <buildtool_depend>catkin</buildtool_depend>
  <depend>rospy</depend>
  <depend>std_msgs</depend>
  <exec_depend>python3</exec_depend>
</package>

## requirements.txt
openai

## launch/chat_node.launch
<launch>
  <node pkg="openai_bot" type="chat_node.py" name="chat_node" output="screen" />
</launch>

## scripts/chat_node.py
#!/usr/bin/env python3
import rospy
import openai
import os
from std_msgs.msg import String

# 使用环境变量获取 API 密钥
openai.api_key = os.getenv("OPENAI_API_KEY")

def callback(msg):
    prompt = msg.data
    rospy.loginfo(f"Received input: {prompt}")

    try:
        response = openai.ChatCompletion.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": "你是一个友好的机器人助手。"},
                {"role": "user", "content": prompt}
            ]
        )
        reply = response['choices'][0]['message']['content']
        rospy.loginfo(f"GPT response: {reply}")
        pub.publish(reply)
    except Exception as e:
        rospy.logerr(f"OpenAI API error: {e}")
        pub.publish("Error contacting OpenAI API")

def listener():
    rospy.init_node('chatgpt_node', anonymous=True)
    rospy.Subscriber("chat_input", String, callback)
    global pub
    pub = rospy.Publisher("chat_output", String, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    listener()

## README.md
# openai_bot

A ROS Noetic package to interact with OpenAI GPT API.

## Setup

```bash
cd ~/catkin_ws/src
git clone <this-repo>
cd ~/catkin_ws
catkin_make
source devel/setup.bash
pip3 install -r src/openai_bot/requirements.txt
```

## Usage

```bash
roslaunch openai_bot chat_node.launch
```

Publish to topic:
```bash
rostopic pub /chat_input std_msgs/String "data: '你好，机器人！'"
```

Echo response:
```bash
rostopic echo /chat_output
```
