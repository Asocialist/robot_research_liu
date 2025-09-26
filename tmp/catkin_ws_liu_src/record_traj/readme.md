# record robot and person position

- start this node when robot starts moving

``` bash
$ rosrun record_traj record_traj.py
$ rostopic echo -n1 /traj/robot_map
$ rostopic echo -n1 /traj/person_map
$ rosbag record -O run1.bag /traj/robot_map /traj/person_map
```

- after recording change rosbag file into csv

``` bash
$ rostopic echo -b run1.bag -p /traj/robot_map  > robot_traj.csv
$ rostopic echo -b run1.bag -p /traj/person_map > person_traj.csv
```

``` bash
rosrun record_traj record_traj.py \
    _is_tracking_global:=false \
    _buffer_len:=200 \
    _max_dt_sec:=0.2 \
    _allow_stale:=false
```