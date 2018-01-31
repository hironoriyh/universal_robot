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

    speed_move = 1.0
    speed_cut = 0.1
    print "============ setup"
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_milling', anonymous=True)
    ## the robot as a whole.
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("manipulator")
    # interface = group.MoveGroupInterface()
    # group.setPlannerId('')
    group.set_planning_time(3)
    group.set_planner_id('RRTkConfigDefault')
    group.allow_replanning(True)
    # group.set_goal_position_tolerance(0.001)
    display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory,
                                    queue_size=20)



    print "current joint values: \n", group.get_current_joint_values()
    print "current pose:   \n", group.get_current_pose().pose
    group_variable_values = [
0.45838239789009094, -1.8014166990863245, -1.5130813757525843, -2.9690874258624476, -1.805875603352682, -3.1426265875445765                             ]
    moveJoint(group, group_variable_values, speed_move)
    move_perpendicular(group, speed_move)
    org_pose = group.get_current_pose().pose
    print " org pos: " , org_pose

    print "============ Going up"
    moveRelativePt(group, [0.0, 0.0, 0.05], speed_move)

    print "============ move to above"
    side_cut_1 =  np.loadtxt('brT/1_sidecut_T1.txt')*0.001
    new_arrays = np.array_split(side_cut_1, len(side_cut_1)/8)

    first_point = new_arrays[0][0]
    point_up = [first_point[0], first_point[1], 0.05] # x, y is swapped
    print point_up
    # point_up = [0.05, 0.0, 0.05] # x, y is swapped
    moveRelRotPt(group, point_up, org_pose, speed_move)
    rospy.sleep(1.0)

    print "============ move down"
    moveRelRotPt(group, [first_point[0], first_point[1], 0.0], org_pose, speed_move)

    print "============ side cut"
    for layer in  new_arrays:
        for pt in layer:
            moveRelRotPt(group, pt, org_pose, speed_cut)
    print "finished!"

# def setio_callback(req):
#     # req

def move_perpendicular(group, speed_move):
    pose_target = group.get_current_pose()
    pose_target.pose.orientation.x = 0.0
    pose_target.pose.orientation.y = 0.0
    pose_target.pose.orientation.z = -0.94
    pose_target.pose.orientation.w = 0.341
    moveAbsPose(group, pose_target, speed_move)
    print "joint values after perpendicular move: \n", group.get_current_joint_values()


def moveJoint(group, group_variable_values, speed):
    group.set_start_state_to_current_state()
    group.set_max_velocity_scaling_factor(speed)
    group.clear_pose_targets()
    print "============ Joint values: ", group_variable_values
    group.set_joint_value_target(group_variable_values)
    plan = group.plan()
    # print plan
    group.go(wait=True)
    # group.execute(plan1)
    rospy.sleep(1)

def moveCartesianPath(group, pts, org_pose, speed, steps):
    # group.set_max_velocity_scaling_factor(speed)
    waypoints = []
    waypoints.append(group.get_current_pose().pose)

    # print "------------------- current pose -----", pose_target.position
    for pt in pts:
        pose_target = copy.deepcopy(org_pose)
        pose_target.position.x -= pt[1]
        pose_target.position.y += pt[0]
        pose_target.position.z += pt[2]
        waypoints.append(pose_target)
        print pose_target.position.x, pose_target.position.y, pose_target.position.z

    (plan, fraction) = group.compute_cartesian_path(
                                 waypoints,   # waypoints to follow
                                 steps,        # eef_step
                                 0.0)         # jump_threshold
    print 'cartesian path start cutting!', fraction
    group.go(wait=True)
    group.execute(plan)
    rospy.sleep(0.5)
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
    # group.set_position_target(0.01, 0.01, 0.1)
    plan = group.plan()
    # print plan
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
    plan = group.plan()
    # print plan
    group.go(wait=True)
    rospy.sleep(1)

def moveAbsPose(group, pose_target, speed):
    # if(speed):
    group.set_max_velocity_scaling_factor(speed)

    group.clear_pose_targets()
    group.set_start_state_to_current_state()
    group.set_pose_target(pose_target)
    plan = group.plan()
    group.go(wait=True)
    rospy.sleep(1)

if __name__=='__main__':
  try:
    milling_paths()
  except rospy.ROSInterruptException:
    pass
