#!/usr/bin/env python

## BEGIN_SUB_TUTORIAL imports
##
## To use the python interface to move_group, import the moveit_commander
## module.  We also import rospy and some messages that we will use.
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

## END_SUB_TUTORIAL

import numpy as np

from std_msgs.msg import String

def move_group_python_interface_tutorial():

    print "============ Starting tutorial setup"
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
    ## the robot as a whole.
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("manipulator")
    display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory,
                                    queue_size=20)

    print "============ Going up"
    moveAbsPt(group, [0.2, 0.2, 0.3], 0.05)

    print "============ Side cut 1"
    side_cut_1 =  np.loadtxt('1_first_sidecut_N.txt')*0.001
    # side_cut_1 += 100
    # np.ndarray( )
    home_pos = group.get_current_pose().pose.position
    print home_pos
    tool_length = 0.15
    for pt in side_cut_1:
        world_point = [home_pos.x + pt[0], home_pos.y + pt[1], pt[2] + tool_length]
        print world_point
        moveAbsPt(group, pt, 0.005)

    moveit_commander.roscpp_shutdown()
    print "============ STOPPING"

def moveAbsPt(group, pt, speed):
    # if(speed):
    group.set_max_velocity_scaling_factor(speed)

    group.clear_pose_targets()
    group.set_start_state_to_current_state()
    pose_target = group.get_current_pose().pose

    pose_target.position.x = pt[0]
    pose_target.position.y = pt[1]
    pose_target.position.z = pt[2]

    group.set_pose_target(pose_target)
    plan1 = group.plan()
    group.go(wait=True)

if __name__=='__main__':
  try:
    move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
    pass
