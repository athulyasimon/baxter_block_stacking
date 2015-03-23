#!/usr/bin/env python

import sys
import copy
import rospy
import geometry_msgs.msg

from std_msgs.msg import (Header, String)
from geometry_msgs.msg import (PoseStamped, Pose, Point, Quaternion)

import moveit_commander
import moveit_msgs.msg

from moveit_commander import MoveGroupCommander
from moveit_msgs.msg import Grasp


if __name__ == '__main__':

    #First initialize moveit_commander and rospy.
    print "============ Starting tutorial setup"
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',anonymous=True)

    #Instantiate a RobotCommander object. This object is an interface to the robot as a whole.
    robot = moveit_commander.RobotCommander()

    #Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
    print "============ Waiting for RVIZ..."
    rospy.sleep(2)

    #Instantiate a PlanningSceneInterface object. This object is an interface to the world surrounding the robot.
    scene = moveit_commander.PlanningSceneInterface()

    #Instantiate a MoveGroupCommander object. This object is an interface to one group of joints. In this case the group is the joints in the left arm. This interface can be used to plan and execute motions on the left arm.
    group = MoveGroupCommander("left_arm")


    #We create this DisplayTrajectory publisher which is used below to publish trajectories for RVIZ to visualize.
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',      moveit_msgs.msg.DisplayTrajectory)


    rospy.sleep(2)
    scene.remove_world_object("part")


    # Publish block 1
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()

    p.pose.position.x = 0.8
    p.pose.position.y = 0.1
    p.pose.position.z = 0.07

    p.pose.orientation.x = 0
    p.pose.orientation.y = 0
    p.pose.orientation.z = 0
    p.pose.orientation.w = 1


    print "=============== Publishing block 1"
    scene.add_box("cube1", p, (0.05, 0.05, 0.05))


    # Publish block 1
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    
    p.pose.position.x = 0.05
    p.pose.position.y = 0
    p.pose.position.z = 0

    p.pose.orientation.x = 0
    p.pose.orientation.y = 0
    p.pose.orientation.z = 0
    p.pose.orientation.w = 1


    print "=============== Publishing pillar"
    scene.add_box("pillar", p, (0.1, 0.1, .07))
    rospy.sleep(1)

    # Publish block 2
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()

    p.pose.position.x = 0.8
    p.pose.position.y = 0.1
    p.pose.position.z = 0.03

    p.pose.orientation.x = 0
    p.pose.orientation.y = 0
    p.pose.orientation.z = 0
    p.pose.orientation.w = 1


    print "=============== Publishing block 2"
    scene.add_box("cube2", p, (0.05, 0.05, 0.05))


    # Publish block 3
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()

    p.pose.position.x = 0.8
    p.pose.position.y = 0.5
    p.pose.position.z = 0.03

    p.pose.orientation.x = 0
    p.pose.orientation.y = 0
    p.pose.orientation.z = 0
    p.pose.orientation.w = 1


    print "=============== Publishing block 3"
    scene.add_box("cube3", p, (0.05, 0.05, 0.05))


    


