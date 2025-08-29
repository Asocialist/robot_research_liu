# 25-8 robot-manual

- to learn how F-function
- check all used package
- check all used commands

## automove and create map

- create map
  - need to run robot twice to get full map  and path data

```bash
sudo ds4drv
ypspur-coordinator -d /dev/ttyACM0 -p ros/icartmini/icart-mini.param
roslaunch launchs/icartmini_hw_proxies.launch
roslaunch launchs/icartmini_controller.launch

//rostopic echo /imu_vecnav
//rostopic echo /scan
//rostopic echo /motor_rate  check the status of sensors 

rosbag record motor_rate scan imu_vecnav //record 1 minute 

// rename data file and move to/mapfilename
//first time  offset.bag
//second time makemap.bag

roslaunch launchs/icartmini_hw_simulation.launch

cd ros/locations/mapfile
rosrun gnd_lssmap_maker gnd_lssmap_maker lssmap_maker.conf
rosbag play offset.bag

roslaunch ros/launchs/icartmini_hw_simulation.launch 
rosrun gnd_lssmap_maker gnd_lssmap_maker lssmap_maker.conf //second
rosbag play makemap.bag
```

- set waypoint and path

  - open gnd_visualizer`rosrun gnd_visualizer gnd_route_editor ros/locations/ mapfilename/route_editor.conf`
  - Ctrl + Click to place a waypoint.
  - Shift + Click to select. Click on empty space to deselect.
  - When two waypoints are selected in order, a path will be drawn between them.
  - Drag to adjust the position of a waypoint or the width of a path.
  - After selecting a waypoint or path, press Delete to remove it.
  - Ctrl + S to save → return to the terminal and press Ctrl + C to exit.
  - Rename new.path to `<map folder name>`.path and move it into the map folder.
  - change the path of ○icartmini_localization.launch

- the files need to change path

  - icartmini_demo_lever_less.launch
  - nkm_destination_queue.conf
  - nkm_follow_tracking.conf
  - OR_person_following_conf_5.json
  - remote_ctrl_queue_stk_dist.launch
  - when you need to use F-info
    - sks_objectpoint_robot_person.cpp
    - find_objectpoint_by_orientation.cpp
    - conf ☓3
    - ros/launchs/remote_ctrl_queue_stk_dist.launch

- all the commands to start the system (include F-info)

- $ sudo ds4drv
- $ ypspur-coordinator -d /dev/ttyACM0 -p ros/icartmini/icart-mini.param
- $ roslaunch ros/launchs/icartmini_demo_lever_less.launch
- $ rosrun icartmini_sbtp icartmini_sbtp ros/locations/ /navigation_sbtp_icartmini01.conf
- $ rosrun nkm_destination_queue nkm_destination_queue ros/locations/ /nkm_destination_queue.conf
- $ roslaunch ros/launchs/urg_tutorial.launch
- $ rosrun person_following person_following_OR ros/icartmini/stk/OR_person_following_conf_5.json
- $ rosrun nkm_follow_tracking nkm_follow_tracking ros/locations/ /nkm_follow_tracking.conf
- $ rosrun hdk_objectpoint_finder find_objectpoint_by_orientation ros/locations/ /find_objectpoint_by_orientation.conf
- $ rosrun sks_objectpoint_robot_person sks_objectpoint_robot_person ros/locations/ /sks_objectpoint_robot_person.conf
