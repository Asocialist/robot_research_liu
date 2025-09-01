# icartmini based robot
## catkin_ws changed by liu 

- changed package file to adapt noeic enviroment 
- fixed  spaths of launch files 

## rosgpt  created by liu

- for using chatgpt api
- roscore
- export OPENAI_API_KEY
- source ~/.bashrc
- rosrun rosgpt rosgpt.py
- rostopic pub /rosgptinput std_msgs/String "data: 'hello introduce yourself'" -1
- rostopic echo /rosgptpublish
