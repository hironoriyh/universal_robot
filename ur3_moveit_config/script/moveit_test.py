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

  ## Instantiate a RobotCommander object.  This object is an interface to
  ## the robot as a whole.
  robot = moveit_commander.RobotCommander()

  ## Instantiate a PlanningSceneInterface object.  This object is an interface
  ## to the world surrounding the robot.
  scene = moveit_commander.PlanningSceneInterface()

  ## Instantiate a MoveGroupCommander object.  This object is an interface
  ## to one group of joints.  In this case the group is the joints in the left
  ## arm.  This interface can be used to plan and execute motions on the left
  ## arm.
  # print moveit_commander.MoveGroupCommander
  group = moveit_commander.MoveGroupCommander("manipulator")


   ## We create this DisplayTrajectory publisher which is used below to publish
## trajectories for RVIZ to visualize.
  display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory,
                                    queue_size=20)

## Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
  print "============ Waiting for RVIZ..."
  rospy.sleep(0.5)
  print "============ Starting tutorial "

  ## Getting Basic Information
  ## ^^^^^^^^^^^^^^^^^^^^^^^^^
  ##
  ## We can get the name of the reference frame for this robot
  print "============ Reference frame: %s" % group.get_planning_frame()

  ## We can also print the name of the end-effector link for this group
  print "============ Reference frame: %s" % group.get_end_effector_link()
  print   group.get_current_pose()

  ## We can get a list of all the groups in the robot
  print "============ Robot Groups:"
  print robot.get_group_names()

  ## Sometimes for debugging it is useful to print the entire state of the
  ## robot.
  print "============ Printing robot state"
  print robot.get_current_state()
  print "============"


  ## Planning to a joint-space goal
  group.clear_pose_targets()
#   group_variable_values = group.get_current_joint_values()
#   print "============ Joint values: ", group_variable_values
#
# ## Now, let's modify one of the joints, plan to the new joint
# ## space goal and visualize the plan
#   group_variable_values = [-0.09089795500040054, -0.22591829299926758, -0.8832510709762573, 0.003594932146370411, -0.22311310470104218, 2.6858160495758057]
#   group.set_joint_value_target(group_variable_values)
#   plan2 = group.plan()
#   print "============ Waiting while RVIZ displays plan2..."
#   rospy.sleep(2)
#   ## You can ask RVIZ to visualize a plan (aka trajectory) for you.  But the
#   ## group.plan() method does this automatically so this is not that useful
#   ## here (it just displays the same trajectory again).
#   print "============ Visualizing plan2"
#   display_trajectory = moveit_msgs.msg.DisplayTrajectory()
#   display_trajectory.trajectory_start = robot.get_current_state()
#   display_trajectory.trajectory.append(plan2)
#   display_trajectory_publisher.publish(display_trajectory);
#   print "============ Waiting while plan2 is visualized (again)..."
#   rospy.sleep(2)
#   group.go(wait=True)
#   group.execute(plan2)

  ## Planning to a Pose goal
  print "============ Generating plan 1"
  group.clear_pose_targets()
#   pose_target = group.get_current_pose().pose
#   pose_target.position.z -= 0.2
  group.set_start_state_to_current_state()
  pose_target = geometry_msgs.msg.Pose()
  pose_target.position.x = -0.30
  pose_target.position.y = -0.26
  pose_target.position.z = 0.12
  pose_target.orientation.x = -0.394
  pose_target.orientation.y = 0.596
  pose_target.orientation.z =  0.347
  pose_target.orientation.w = 0.606
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

  print "============ Generating plan 1"
  group.clear_pose_targets()
  group.set_start_state_to_current_state()
  pose_target = group.get_current_pose().pose
  pose_target.position.x -= 0.1
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

  ## Cartesian Paths
  ## ^^^^^^^^^^^^^^^
  ## You can plan a cartesian path directly by specifying a list of waypoints
  ## for the end-effector to go through.
  waypoints = []

  # start with the current pose
  waypoints.append(group.get_current_pose().pose)

  print "waypoints"
  print waypoints

  # first orient gripper and move forward (+x)
  wpose = geometry_msgs.msg.Pose()
  wpose.position.x = waypoints[0].position.x + 0.1
  waypoints.append(copy.deepcopy(wpose))

  # second move down
  wpose.position.z -= 0.10
  waypoints.append(copy.deepcopy(wpose))

  # third move to the side
  wpose.position.y += 0.05
  waypoints.append(copy.deepcopy(wpose))

  ## disabling it.
  (plan3, fraction) = group.compute_cartesian_path(
                               waypoints,   # waypoints to follow
                               0.001,        # eef_step
                               0.0)         # jump_threshold

  print "============ Waiting while RVIZ displays plan3..."
  rospy.sleep(3)
  print "============ Visualizing plan3"
  display_trajectory = moveit_msgs.msg.DisplayTrajectory()
  display_trajectory.trajectory_start = robot.get_current_state()
  display_trajectory.trajectory.append(plan3)
  display_trajectory_publisher.publish(display_trajectory);
  rospy.sleep(2)
  group.go(wait=True)
  group.execute(plan3)

  collision_object = moveit_msgs.msg.CollisionObject()


  moveit_commander.roscpp_shutdown()
  print "============ STOPPING"


if __name__=='__main__':
  try:
    move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
    pass
