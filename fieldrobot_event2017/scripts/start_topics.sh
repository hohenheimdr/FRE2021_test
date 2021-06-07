#!/bin/bash
#publish speed msg to get robot to start position
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" -1 

#publish joy button A to get into row navigation mode
rostopic pub /joy/button_A std_msgs/Bool "data: true" -1 &


