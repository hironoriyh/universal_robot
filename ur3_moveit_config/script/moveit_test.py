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
import copy
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

    print "============ joint move "
    # group_variable_values = [-5.64, -1.60, -1.61, 3.17, -0.90, -3.13]
    group_variable_values = [-5.342680160199301, -1.7783635298358362, -1.498627010975973, 3.2526774406433105, -0.3909981886493128, -3.180640999470846]
    moveJoint(group, group_variable_values, 0.05)

    print "============ Going up"
    moveRelativePt(group, [0.0, 0.0, 0.01], 0.05)
    # moveRelativePt(group, [-0.08, -0.0, 0.0172100525103], 0.01)

    print "============ Side cut 1"
    side_cut_1 =  np.loadtxt('1_first_sidecut_T1.txt')*0.001
    print side_cut_1[:10]
    org_pose = group.get_current_pose()
    print " home pos: " , org_pose
    tool_length = 0.15
    for pt in side_cut_1:
        # world_point = [org_pose.x + pt[0], org_pose.y + pt[1], pt[2]]
        print "move relative to: ",  pt
        moveRelRotPt(group, pt, org_pose, 0.05)

    moveit_commander.roscpp_shutdown()
    print "============ STOPPING"

def moveJoint(group, group_variable_values, speed):
    group.set_start_state_to_current_state()
    group.set_max_velocity_scaling_factor(speed)
    group.clear_pose_targets()
    print "============ Joint values: ", group_variable_values
    group.set_joint_value_target(group_variable_values)
    plan1 = group.plan()
    group.go(wait=True)
    # group.execute(plan1)
    rospy.sleep(1)

def moveRelRotPt(group, pt, org_pose, speed):
    # if(speed):
    group.set_max_velocity_scaling_factor(speed)
    group.clear_pose_targets()
    group.set_start_state_to_current_state()
    pose_target = copy.deepcopy(org_pose)
    print "--------rel move -----", pt
    # print "------------------- current pose -----", pose_target.position
    pose_target.pose.position.x += pt[1]
    pose_target.pose.position.y += pt[0]
    pose_target.pose.position.z += pt[2]
    # print "------------------- after update pose -----", pose_target.position

    group.set_pose_target(pose_target)
    plan1 = group.plan()
    group.go(wait=True)
    rospy.sleep(1)

def moveRelativePt(group, pt, speed):
    # if(speed):
    group.set_max_velocity_scaling_factor(speed)
    group.clear_pose_targets()
    group.set_start_state_to_current_state()
    pose_target = group.get_current_pose().pose
    print "--------rel move -----", pt
    # print "------------------- current pose -----", pose_target.position
    pose_target.position.x += pt[0]
    pose_target.position.y += pt[1]
    pose_target.position.z += pt[2]
    # print "------------------- after update pose -----", pose_target.position

    group.set_pose_target(pose_target)
    plan1 = group.plan()
    group.go(wait=True)
    rospy.sleep(1)

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
    rospy.sleep(1)

if __name__=='__main__':
  try:
    move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
    pass
