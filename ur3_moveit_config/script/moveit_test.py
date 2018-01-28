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
from ur_msgs.srv import SetIO
## END_SUB_TUTORIAL

import numpy as np

from std_msgs.msg import String

def milling_paths():

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

    print "current joint values:  ", group.get_current_joint_values()
    group_variable_values = [-5.554026071225302, -1.7539408842669886, -1.6314790884601038, 3.347646713256836, -0.9896319548236292, -3.108539406453268]
    # group_variable_values = [-5.331279043351309, -1.381209675465719, -2.1003029982196253, 3.4650821685791016, -0.5601938406573694, -3.156151835118429]
    moveJoint(group, group_variable_values, 0.05)
    org_pose = group.get_current_pose().pose
    print " org pos: " , org_pose

    print "============ Going up"
    moveRelativePt(group, [0.0, 0.0, 0.05], 0.05)

    print "============ Side cut 1"
    side_cut_1 =  np.loadtxt('brT/2_second_sidecut_T1.txt')*0.001
    point_up = [side_cut_1[0][0], side_cut_1[0][1], 0.05] # x, y is swapped
    moveRelRotPt(group, point_up, org_pose, 0.05)
    rospy.sleep(1.0)

    moveRelRotPt(group, side_cut_1[0], org_pose, 0.05)
    for pt in  side_cut_1:
        moveRelRotPt(group, pt, org_pose, 0.01)
    print "finished!"
# def setio_callback(req):
#     # req

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

def moveCartesianPath(group, pts, org_pose, speed, steps):
    group.set_max_velocity_scaling_factor(speed)
    waypoints = []
    waypoints.append(group.get_current_pose().pose)

    # print "------------------- current pose -----", pose_target.position
    for pt in pts:
        pose_target = copy.deepcopy(org_pose)
        pose_target.position.x += pt[1]
        pose_target.position.y += pt[0]
        pose_target.position.z += pt[2]
        waypoints.append(pose_target)
        print pose_target.position.z

    (plan, fraction) = group.compute_cartesian_path(
                                 waypoints,   # waypoints to follow
                                 steps,        # eef_step
                                 0.0)         # jump_threshold
    # group.go(wait=True)
    print 'cartesian path start cutting!'
    # group.go(wait=True)
    group.execute(plan)
    rospy.sleep(1)
    print 'cartesian path finished!'

def moveRelRotPt(group, pt, org_pose, speed):
    # if(speed):
    group.set_max_velocity_scaling_factor(speed)
    group.clear_pose_targets()
    group.set_start_state_to_current_state()
    pose_target = copy.deepcopy(org_pose)
    print "--------rel move -----", pt
    # print "------------------- current pose -----", pose_target.position
    pose_target.position.x -= pt[1]
    pose_target.position.y += pt[0]
    pose_target.position.z += pt[2]
    # print "------------------- after update pose -----", pose_target.position
    pose_target_stamped = group.get_current_pose()
    pose_target_stamped.pose = pose_target
    group.set_pose_target(pose_target)
    plan1 = group.plan()
    group.go(wait=True)
    rospy.sleep(0.01)

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
    milling_paths()
  except rospy.ROSInterruptException:
    pass
