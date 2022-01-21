#!/usr/bin/env python
import time
from geometry_msgs.msg import Point
from pilz_robot_programming import *
import math
import moveit_commander
import rospy

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
current_pose = move_group.get_current_pose().pose
                        
pose_goal = current_pose
while True:
    sequence = Sequence()
    moves = [(0,0,.20),(0,.2,0),(0,0,-0.2),(0,-.2,0)]

    for x, y, z in moves:
        r.move(Ptp(goal=Pose(position=Point(x, y, z)), relative=True,
           vel_scale=0.1, acc_scale=0.1, planning_group="arm",target_link="Face_Plate_1",reference_frame="base_link"))
        # sequence.append(Ptp(goal=pose_goal, planning_group="arm",target_link="Face_Plate_1",reference_frame="base_link"))

    print("Starting")
    # r.move(sequence)
    time.sleep(4)
