#!/usr/bin/env python
import time
from geometry_msgs.msg import Point
from pilz_robot_programming import *
import math
import moveit_commander
import rospy
import numpy as np

__REQUIRED_API_VERSION__ = "1"
_DEFAULT_PLANNING_GROUP = "arm" 
_DEFAULT_TARGET_LINK = "Face_Plate_1"
_DEFAULT_GRIPPER_PLANNING_GROUP = "gripper"
_DEFAULT_BASE_LINK = "base_link"

rospy.init_node('robot_program_node')
print("Executing " + __file__)

r = Robot(__REQUIRED_API_VERSION__)
group_name = "arm"
move_group = moveit_commander.MoveGroupCommander(group_name)

cut_home = Ptp(goal=np.deg2rad([27,-19,-3,144,34,139]), vel_scale=0.05, acc_scale=0.05, planning_group="arm",target_link="Face_Plate_1",reference_frame="base_link")

current_pose = move_group.get_current_pose().pose                        
pose_goal = current_pose

while True:
    sequence = Sequence()
    r.move(cut_home)
    step_size = 0.0254
    step_length = step_size
    x, y = 0, 0
    i = 0
    for _ in range(5):
        move = [0, 0, 0]
        if i % 2 == 0:
            if i > 1:
                move[0] = step_length
            else:
                move[0] = -step_length
        else:
            if i > 1:
                move[1] = step_length
            else:
                move[1] = -step_length
        i += 1
        i %= 4
        step_length += step_size 
        print(move)
        cut = Lin(goal=Pose(position=Point(*move)), relative=True,
            vel_scale=0.01, acc_scale=0.01, planning_group="arm",target_link="Face_Plate_1",reference_frame="base_link")
        sequence.append(cut)
    print("Starting")
    r.move(sequence)
    time.sleep(4)
