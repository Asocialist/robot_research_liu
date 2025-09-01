#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import openai
import os # 导入 os 模块

def handle_input(msg):
    """
    处理从 /llm_input 话题接收到的消息，并调用 OpenAI API。
    """
    global chat_history

    # 检查输入消息是否有效
    user_msg = msg.data
    if not user_msg:
        rospy.logwarn("接收到空消息，已忽略。")
        return

    rospy.loginfo("user input: %s", user_msg)

    # 更新历史信息
    chat_history.append({"role": "user", "content": user_msg})

    # 限制历史记录的长度，防止上下文过长导致 token 超出限制
    if len(chat_history) > 10:
        chat_history = chat_history[-10:]

    try:
        # 调用 OpenAI API
        response = openai.ChatCompletion.create(
            model="gpt-3.5-turbo",  # 或者更新的模型如 "gpt-4o"
            messages=chat_history
        )
        # 提取回复内容
        reply = response.choices[0].message['content']
        rospy.loginfo("ChatGPT reply: %s", reply)

        # 将模型的回复也加入历史记录，以实现连续对话
        chat_history.append({"role": "assistant", "content": reply})

        # 发布回复
        pub.publish(reply)

    except openai.error.OpenAIError as e:
        rospy.logerr("ChatGPT API call failed: %s", str(e))
        pub.publish("系统错误：无法联系 ChatGPT。请检查日志。")
    except Exception as e:
        rospy.logerr("发生未知错误: %s", str(e))
        pub.publish("系统错误：发生未知错误。")

if __name__ == "__main__":
    rospy.init_node("rosgpt_node") # 节点名称建议和文件名或功能匹配

    # --- 安全地加载 API 密钥 ---
    # 强烈建议使用环境变量来存储 API 密钥
    api_key = os.getenv("OPENAI_API_KEY")
    if api_key is None:
        rospy.logerr("错误：未设置 OPENAI_API_KEY 环境变量。")
        rospy.logerr("请在终端执行: export OPENAI_API_KEY='你的sk-密钥'")
        exit(1) # 如果没有密钥则退出程序
    openai.api_key = api_key
    
    # 初始化会话历史
    chat_history = []

    # 定义发布者和订阅者
    pub = rospy.Publisher("/rosgptpublish", String, queue_size=10)
    sub = rospy.Subscriber("/rosgptinput", String, handle_input, queue_size=1) # queue_size=1 确保处理最新消息

    rospy.loginfo("rosgpt started listening /rosgptinput topic...")

    # 保持节点运行
    rospy.spin()