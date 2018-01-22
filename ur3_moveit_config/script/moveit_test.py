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
  group.set_max_velocity_scaling_factor(0.1) 

#   group.set_max_acceleration_scaling_factor(0.1)

  ## Planning to a Pose goal
  print "============ Generating plan 1"
  group.clear_pose_targets()
  group.set_start_state_to_current_state()
  pose_target = geometry_msgs.msg.Pose()
  pose_target.position.x = -0.30
  pose_target.position.y = -0.197
  pose_target.position.z = 0.28
  pose_target.orientation.x = -0.040
  pose_target.orientation.y = 0.002
  pose_target.orientation.z =  -0.698
  pose_target.orientation.w = 0.714
  
  group.set_pose_target(pose_target)
  plan1 = group.plan()
  print "============ Waiting while RVIZ displays plan1..."
  rospy.sleep(3)
  print "============ Visualizing plan1"
  display_trajectory = moveit_msgs.msg.DisplayTrajectory()
  display_trajectory.trajectory_start = robot.get_current_state()
  display_trajectory.trajectory.append(plan1)
  display_trajectory_publisher.publish(display_trajectory);
  rospy.sleep(2)
  group.go(wait=True)
  group.execute(plan1)

  group.set_max_velocity_scaling_factor(0.1) 


  print "============ Generating plan 2"
  for i in range(3):
      group.clear_pose_targets()
      group.set_start_state_to_current_state()
      pose_target.position.x += 0.05
      print pose_target
      group.set_pose_target(pose_target)
      plan1 = group.plan()
      group.go(wait=True)
      group.execute(plan1)
      
      rospy.sleep(0.5)
      group.clear_pose_targets()
      group.set_start_state_to_current_state()
      pose_target.position.y -= 0.05
      print pose_target
      group.set_pose_target(pose_target)
      plan1 = group.plan()
      group.go(wait=True)
      group.execute(plan1)



  moveit_commander.roscpp_shutdown()
  print "============ STOPPING"


if __name__=='__main__':
  try:
    move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
    pass
