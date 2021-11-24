#!/bin/bash
sleep 10

rostopic pub -1 /move_base/goal move_base_msgs/MoveBaseActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  target_pose:
    header:
      seq: 0
      stamp:
        secs: 0
        nsecs: 0
      frame_id: 'map'
    pose:
      position:
        x: 1.0
        y: 7.0
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 1.0
        w: 0.0" 