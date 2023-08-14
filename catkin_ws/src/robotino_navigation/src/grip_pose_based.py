#! /usr/bin/env python3

# from nntplib import GroupInfo
import sys
# import copy
import rospy
import moveit_commander
import moveit_msgs.msg
# import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from numpy import * 
# import numpy as np

# Length of links in cm

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial',anonymous=True)

robot=moveit_commander.RobotCommander()

scene=moveit_commander.PlanningSceneInterface()
group=moveit_commander.MoveGroupCommander("arm")
gripper=moveit_commander.MoveGroupCommander("gripper")

display_trajectory_publisher=rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory, queue_size=100)

gripper.set_named_target ("open")
gripper.go(wait=True)

group.set_named_target ("home")
group.go(wait=True)

group.set_named_target ("pregrip")
group.go(wait=True)

gripper.set_named_target ("closed")
gripper.go(wait=True)

group.set_named_target ("initial")
group.go(wait=True)


moveit_commander.roscpp_shutdown()
