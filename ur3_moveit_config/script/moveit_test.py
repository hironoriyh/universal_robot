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
    print "============ Generating plan 1"
    group.clear_pose_targets()
    group.set_start_state_to_current_state()
    pose_target = group.get_current_pose().pose
    pose_target.position.z += 0.05
    group.set_pose_target(pose_target)
    plan1 = group.plan()
    group.go(wait=True)
    rospy.sleep(3)

 ## Planning to a Pose goal
    print "============ Generating plan 2"
    group.clear_pose_targets()
    group_variable_values = group.get_current_joint_values()
    print "============ Joint values: ", group_variable_values
    group_variable_values = [-5.64, -1.60, -1.61, 3.17, -0.90, -3.13]
    # group_variable_values = [-5.646939400826589, -1.6089308897601526, -1.8123844305621546, 3.42496919631958, -0.8998850027667444, -3.1375396887408655]
    # group_variable_values[0] = -5.7
    group.set_joint_value_target(group_variable_values)
    plan1 = group.plan()
    group.go(wait=True)
    rospy.sleep(1)

    group.set_max_velocity_scaling_factor(0.005)
    rospy.sleep(3)
    print "============ Generating plan 3"
    pose_target = group.get_current_pose().pose
    for i in range(1):
        group.clear_pose_targets()
        group.set_start_state_to_current_state()
        pose_target.position.x += 0.01
        print pose_target.position.x
        group.set_pose_target(pose_target)
        plan1 = group.plan()
        group.go(wait=True)

        rospy.sleep(0.5)
        group.clear_pose_targets()
        group.set_start_state_to_current_state()
        pose_target.position.y -= 0.002
        print pose_target.position.y
        group.set_pose_target(pose_target)
        plan1 = group.plan()
        group.go(wait=True)

        group.clear_pose_targets()
        group.set_start_state_to_current_state()
        pose_target.position.x -= 0.01
        print pose_target.position.x
        group.set_pose_target(pose_target)
        plan1 = group.plan()
        group.go(wait=True)

        rospy.sleep(0.5)
        group.clear_pose_targets()
        group.set_start_state_to_current_state()
        pose_target.position.y -= 0.002
        print pose_target.position.y
        group.set_pose_target(pose_target)
        plan1 = group.plan()
        group.go(wait=True)


    moveit_commander.roscpp_shutdown()
    print "============ STOPPING"


if __name__=='__main__':
  try:
    move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
    pass
