Baxter Block Stacking with MoveIt!
=============================================

Athulya Simon 
---------------------------------------------


#### Table of Contents ####
[Project Overview](#Project Overview)

[Requirements and Helpful Tutorials](#Requirements)

[Block Detection with OpenCV](#Vision)

[Block Pick Up and Drop Off with MoveIt!](#Movement)

[Collision Objects](#Collision)

[Future Work](#Future Work)

[General Debugging](#Debugging)

[Instructions for running the files](#Instructions)


#### Project Overview  <a name="Project Overview"></a>
The main goal of this project was to gain more experience with the Baxter robot from Rethinks Robotics and the MoveIt! package which was originally developed by Willow Garage.The task of block stacking was chosen to explore the features of motion planning while under the constraints of avoiding certain objects. OpenCV was also used to aid with the detection of blocks. Here are the project goals broken down:
  *  Locate blocks using OpenCV
  *  Pick up blocks and move to block tower location using MoveIt!
  *  Make use of Collision Objects to avoid knocking over blocks during movement

#### Requirements and Helpful Tutorials  <a name="Requirements"></a>

######Requirements
  *  Baxter SDK - follow the [Workstation Setup] (http://sdk.rethinkrobotics.com/wiki/Workstation_Setup) Instructions
  *  MoveIt! - this can be obtained through `sudo apt-get install ros-indigo-moveit-full`
  *  You will also need Gazebo (`sudo apt-get install gazebo5`) if you want to work with simulated Baxter

######Tutorials

1. [Baxter Setup](http://sdk.rethinkrobotics.com/wiki/Baxter_Setup) instructions,  [Workstation Setup](http://sdk.rethinkrobotics.com/wiki/Workstation_Setup) instructions and basic [Hello Baxter!](http://sdk.rethinkrobotics.com/wiki/Hello_Baxter) example demo.   
2. [Rethink Robotics Baxter MoveIt Tutorial](http://sdk.rethinkrobotics.com/wiki/MoveIt_Tutorial#Tutorial)
3. [Move Group Interface Python](http://docs.ros.org/hydro/api/pr2_moveit_tutorials/html/planning/scripts/doc/move_group_python_interface_tutorial.html)
4. [Gazebo Tutorials](http://gazebosim.org/tutorials) are helpful with creating your own simulation environment

#### Block Detection with OpenCV  <a name="Vision"></a>
The location of the block must first be determined before the block can be picked up. Each block used for stacking has a green piece of tape on it that is used for detection. The camera feed from Baxter’s left arm is thresholded against the HSV values for green to find objects. Once the green is detected the x and y coordinates of that object are published to the /opencv/center_of_object topic. An [Image Pixel to Workspace coordinate conversion](http://sdk.rethinkrobotics.com/wiki/Worked_Example_Visual_Servoing) was used to determine the offset between the actual position of the object and the coordinates calculated from the camera feed.



#### Block Pick Up and Drop Off with MoveIt!  <a name="Movement"></a>
The accuracy of block pick up and drop off varies with the number of blocks in each tower; about two blocks high is usually the best. Although the waypoints and drop off points are hard coded, MoveIt sometimes fails to find a path between a waypoint and drop off point. Waypoints were added to prevent Baxter from knocking over the blocks while the arm is in motion. This strategy is to be replaced in the future with the use of published collision objects.


#### Collision Objects <a name="Collision"></a>
The blocks can be displayed in Rviz using the file collision_object.py, however they are just displayed there and MoveIt! makes no attempt to avoid these objects. I have tried publishing the current scene (as when following the [MoveIt tutorial](https://github.com/RethinkRobotics/sdk-docs/wiki/MoveIt-Tutorial) with the pillar object), but that doesn't seem to work. The file move_arm_place_collision.py is my attempt to publish the blocks while MoveIt! is planning the paths. The file collision_object.cpp seems to have the proper methods for publishing a scene, but there are issues with actually running the file. Adding moveit_ros_planning_interface in my CMakeLists seems to have fixed the problem of "eigen/geometry no such file or directory" during catkin_make, but now I have the problem "/bin: Is a directory" and the lines of code that it points to are comments.  

#### Future Work  <a name="Future Work"></a>
######Block Detection
With Block Detection I would like to increase the area from which the block's coordinates can be correctly detected. I would also like to incorporate the use of a second camera to also have reliable information on the height of the object.

######Collision Objects
In the near future, I would also like to get the blocks properly published as collision objects. This will eliminate the need for waypoints and will also result in much fewer instances of Baxter knocking the blocks onto the floor. 


#### General Debugging  <a name="Debugging"></a>
######General Baxter Tips:

Always start with 
* sudo service network-manager stop
* sudo avahi-autoipd eth0

Make sure that roscore is running on Baxter for each terminal window that you open using the command `. baxter.sh` 

If the error “Couldn’t find an AF_INET address for …” comes up, try turning Baxter off and then back on. 

######General MoveIt! Tips:

MoveIt! is a very intertwined package, and in order to make everything work there are a few things that must also be running. The easiest method that I found, which includes everything that you need, was to run the following two files together:
* `rosrun baxter_interface joint_trajectory_action_server.py`
* `roslaunch baxter_moveit_config demo_baxter.launch` 
Wait for the green phrase “All is well, Everyone is Happy, You can start planning now!” before using Rviz or running any other files. 


#### Instruction for running files (will eventually be turned into a working launch file)  <a name="Instructions"></a>
Terminal windows 3 and onwards have been placed into a launch file, however a bug in the move_arm_pick.py file prevents a complete execution of all the steps. 

Terminal window 1
* sudo service network-manager stop
* sudo avahi-autoipd eth0

For terminal Window 2 and onwards please do these steps first
* cd baxter_ws/ (new tabs created after this step should already be in the correct folder)
* ./baxter.sh

Terminal window 2
* rosrun baxter_tools enable_robot.py -e
* rosrun baxter_interface joint_trajectory_action_server.py

Terminal window 3
* roslaunch baxter_moveit_config demo_baxter.launch

Wait for the green phrase "All is well, Everyone is Happy, You can start planning now!"

Terminal window 4
* rosrun baxter_block_stack move_arm_vision.py
* rosrun baxter_block_stack open_cv_vision.py

Terminal window 5
* rosrun baxter_block_stack move_arm_pick.py
* rosrun baxter_block_stack move_arm_place.py
