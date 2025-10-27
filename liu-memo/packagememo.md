# package-memo 

- take a memo of the packages that are used
- to understand the structure of package needs to be created

-----


## ros_gpt

- 与gpt的交互实现

``` python
def _chat_reply_with_context(self, user_text):
    # 1. 构造上下文提示词
    system_prompt = f"""
    You are a robot assistant. 
    Robot pose: {self.robot_pose}
    Person pose: {self.person_pose}
    Reply in JSON format with keys 'reply_for_user' and 'navigation_command'.
    """

    # 2. 调用 OpenAI ChatCompletion API
    response = self.client.chat.completions.create(
        model=self.model_name,
        messages=[
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_text}
        ],
        temperature=0.3
    )

    # 3. 解析结果
    message = response.choices[0].message.content.strip()
    return json.loads(message)
```







## person_following



## sy_rollsign

## sy_traffic_indicator

