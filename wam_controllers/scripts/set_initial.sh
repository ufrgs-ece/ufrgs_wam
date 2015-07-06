#!/bin/bash

rosservice call /gazebo/set_model_configuration wam joint \
"['wam_joint_1','wam_joint_2','wam_joint_3','wam_joint_4', \
'wam_joint_5','wam_joint_6','wam_joint_7']" \
"[0.0,0.75,0.0,1.5,0.0,0.9,0.0]"

rostopic pub /wam/computed_torque_controller/command \
trajectory_msgs/JointTrajectoryPoint \
"[0.0,0.75,0.0,1.5,0.0,0.9,0.0]" \
"[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]" \
"[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]" \
"[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]" \
"[0.0, 0.0]" -1
