#!/usr/bin/env python
import time
from geometry_msgs.msg import Point
from pilz_robot_programming import Robot, Lin, Pose, Ptp, Sequence
import rospy
import numpy as np

__REQUIRED_API_VERSION__ = "1"

rospy.init_node('robot_program_node')
print("Executing " + __file__)

r = Robot(__REQUIRED_API_VERSION__)

cut_home = Ptp(goal=np.deg2rad([27,-17,0,150,31,146]), vel_scale=1, acc_scale=1, planning_group="arm",target_link="Face_Plate_1",reference_frame="base_link")
one_inch = 0.0254
cut_depth = -one_inch * 1.45
tool_width = one_inch /2

common_args = { 'planning_group':"arm",'target_link':"Face_Plate_1", "vel_scale":.01, "reference_frame" : "base_link"}

push_in = Lin(goal= Pose(position=Point(0,0,cut_depth)), relative = True, **common_args)
pull_out = Lin(goal= Pose(position=Point(0,0,-cut_depth)), relative = True, **common_args)
robot_moves = [] 
robot_moves.append(cut_home)
robot_moves.append(push_in)

max_width = one_inch * 3
step_size = tool_width/2 # Cut with half the tool 
for size in np.arange(0, max_width, step_size):
    moves = [[size,0,0],[0,size,0],[-size,0,0],[0 ,-size,0],[-step_size/2,-step_size/2,0]]#[[x,y,z] ] relative_commands

    for (x,y,z) in moves:
        cut_move = Lin(goal=Pose(position=Point(x,y,z)),relative = True,**common_args)
        robot_moves.append(cut_move)

robot_moves.append(pull_out)
robot_moves.append(cut_home)

raw_input("Press ENTER to run cutting sequence")
time.sleep(3)
for move in robot_moves:
    r.move(move)
#.move(sequence)
