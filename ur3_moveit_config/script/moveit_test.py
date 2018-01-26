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
    robot =
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("manipulator")
    display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory,
                                    queue_size=20)

    print "============ Waiting for RVIZ..."
    rospy.sleep(0.5)
    print "============ Starting tutorial "

    ## Getting Basic Information
    print "============ Reference frame: %s" % group.get_planning_frame()
    print "============ Reference frame: %s" % group.get_end_effector_link()
    print   group.get_current_pose()
    print "============ Robot Groups:"
    print robot.get_group_names()
    print "============ Printing robot state"
    print robot.get_current_state()
    print "============"
    group.set_max_velocity_scaling_factor(0.05)
    #   group.set_max_acceleration_scaling_factor(0.1)

    # Planning to a Pose goal
    print "============ Going up"
    moveRelativePt([0.0, 0, 0.2], 0.05)
    print "============ Side cut 1"
    side_cut_1 =  np.loadtxt('1_first_sidecut_N.txt')
    for pt in side_cut_1:
        moveRelativePt(pt)

    moveit_commander.roscpp_shutdown()
    print "============ STOPPING"
class MoveMilling:
    def __init__(self, move_group, robot, scene):
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("manipulator")

    def moveRelativePt(self, pt, speed):
        if(speed):
            self.group.set_max_velocity_scaling_factor(0.005)

        self.group.clear_pose_targets()
        self.group.set_start_state_to_current_state()
        pose_target = self.group.get_current_pose().pose

        pose_target.position.x += pt[0]
        pose_target.position.y += pt[1]
        pose_target.position.z += pt[2]

        self.group.set_pose_target(pose_target)
        plan1 = self.group.plan()
        self.group.go(wait=True)

    def moveAbsPt(self, pt, speed):
        if(speed):
            self.group.set_max_velocity_scaling_factor(speed)

        self.group.clear_pose_targets()
        self.group.set_start_state_to_current_state()
        pose_target = self.group.get_current_pose().pose

        pose_target.position.x = pt[0]
        pose_target.position.y = pt[1]
        pose_target.position.z = pt[2]

        self.group.set_pose_target(pose_target)
        plan1 = self.group.plan()
        self.group.go(wait=True)

if __name__=='__main__':
  try:
    move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
    pass
