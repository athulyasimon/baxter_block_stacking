#!/usr/bin/env python

# https://github.com/ros-planning/moveit_pr2/blob/hydro-devel/pr2_moveit_tutorials/planning/scripts/move_group_python_interface_tutorial.py

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley

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
import baxter_interface
## END_SUB_TUTORIAL

from std_msgs.msg import (Header, String)
from geometry_msgs.msg import (PoseStamped, Pose, Point, Quaternion)

from moveit_commander import MoveGroupCommander

def move_group_python_interface(block_pos):

  #block_pos = rospy.Subscriber("/opencv/center_of_object", Point)
  
  ## BEGIN_TUTORIAL
  ##
  ## Setup
  ## ^^^^^
  ## CALL_SUB_TUTORIAL imports
  ##
  ## First initialize moveit_commander and rospy.
  print "============ Starting Move Arm for Block Pick Up"
  moveit_commander.roscpp_initialize(sys.argv)
  #rospy.init_node('move_group_python_interface', anonymous=True)

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
  group = MoveGroupCommander("left_arm")


  ## We create this DisplayTrajectory publisher which is used below to publish
  ## trajectories for RVIZ to visualize.
  #display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)

  ## Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
  #print "============ Waiting for RVIZ..."
  #rospy.sleep(5)
  # print "============ Starting tutorial "


  ## Planning to a Pose goal
  ## ^^^^^^^^^^^^^^^^^^^^^^^
  ## We can plan a motion for this group to a desired pose for the 
  ## end-effector
  print "============ Generating plan 1"
  pose_target = geometry_msgs.msg.Pose()
  pose_target.orientation.x = 1 
  #pose_target.orientation.y = 0 
  #pose_target.orientation.z = 0 
  #pose_target.orientation.w = 0 
  pose_target.position.x = block_pos.x 
  pose_target.position.y = block_pos.y 
  pose_target.position.z = -0.05 #0.04 for 3 blocks, 0.0 for 2 blocks, -.05 for 1 block
  group.set_pose_target(pose_target)
  print block_pos.x
  print block_pos.y

  ## Now, we call the planner to compute the plan
  ## and visualize it if successful
  ## Note that we are just planning, not asking move_group 
  ## to actually move the robot
  plan1 = group.plan()

#  print "============ Waiting while RVIZ displays plan1..."
#  rospy.sleep(5)

 
#  ## You can ask RVIZ to visualize a plan (aka trajectory) for you.  But the
#  ## group.plan() method does this automatically so this is not that useful
#  ## here (it just displays the same trajectory again).
#  print "============ Visualizing plan1"
#  display_trajectory = moveit_msgs.msg.DisplayTrajectory()

#  display_trajectory.trajectory_start = robot.get_current_state()
#  display_trajectory.trajectory.append(plan1)
#  display_trajectory_publisher.publish(display_trajectory);

  # print "============ Waiting while plan1 is visualized (again)..."
  # rospy.sleep(5)


#   ## Moving to a pose goal
#   ## ^^^^^^^^^^^^^^^^^^^^^
#   ##
#   ## Moving to a pose goal is similar to the step above
#   ## except we now use the go() function. Note that
#   ## the pose goal we had set earlier is still active 
#   ## and so the robot will try to move to that goal. We will
#   ## not use that function in this tutorial since it is 
#   ## a blocking function and requires a controller to be active
#   ## and report success on execution of a trajectory.

  #Calibrate left gripper
  leftgripper = baxter_interface.Gripper('left')
  # leftgripper.calibrate()
  # leftgripper.open()

#   # Uncomment below line when working with a real robot
  group.go(wait=True)
  rospy.sleep(3)

  #Close Baxter's left gripper
  #leftgripper = baxter_interface.Gripper('left')
  leftgripper.close()

 
#   ## Adding/Removing Objects and Attaching/Detaching Objects
#   ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
#   ## First, we will define the collision object message
  collision_object = moveit_msgs.msg.CollisionObject()

  ## When finished shut down moveit_commander.
  moveit_commander.roscpp_shutdown()

  ## END_TUTORIAL

  print "============ STOPPING"


def target_pose_listener():
  rospy.init_node('target_pose_listener',anonymous = True)
  block_pos = rospy.Subscriber("/opencv/center_of_object", Point, move_group_python_interface)
  rospy.spin()

if __name__=='__main__':
  try:
   count = 1
   if count == 1:
     target_pose_listener()
   else:
     pass
   #move_group_python_interface() 

  except rospy.ROSInterruptException:
    pass
