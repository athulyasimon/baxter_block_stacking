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


#### Project Overview  <a name="Project Overview"></a>
The main goal of this project was to gain more experience with the Baxter robot from Rethinks Robotics and the MoveIt! package, originally developed by Willow Garage.The task of block stacking was chosen to explore the features of motion planning while under the constraints of avoiding certain objects. OpenCV was also used to aid with the detection of blocks. Here are the project goals broken down:
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
The location of the block must first be determined before the block can be picked up. Each block used for stacking has a green piece of tape on it that is used for detection. The camera feed from Baxter’s left arm is thresholded against the HSV values for green to find objects. Once the green is detected the x and y coordinates of that object are published to the /opencv/center_of_object topic. An [Image Pixel to Workspace coordinate conversion](http://sdk.rethinkrobotics.com/wiki/Worked_Example_Visual_Servoing) was used to determine the offset between the actual position of the object and the coordinates that are being published.



#### Block Pick Up and Drop Off with MoveIt!  <a name="Movement"></a>



#### Collision Objects <a name="Collision"></a>




#### Future Work  <a name="Future Work"></a>



#### General Debugging  <a name="Debugging"></a>
######General Baxter Tips:

Always start with 
* sudo service network-manager stop
* sudo avahi-autoipd eth0

Make sure that roscore is running on Baxter for each terminal window that you open using the command `. baxter.sh` 

If the error “Couldn’t find an AF_INET address for …” comes up, try turning Baxter off and then back on. 

######General MoveIt! Tips:

MoveIt! is a very intertwined package, and in order to make everything work there are a few things that must also be running. The easiest method that I found, which includes everything that you need, was to rosrun baxter_interface joint_trajectory_action_server.py along with roslaunch baxter_moveit_config demo_baxter.launch. Wait for the green phrase “All is Happy, Everything is Set Up, You can start planning now” before using Rviz or running any other files. 


####Instruction for running files (will eventually be turned into a launch file):

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

Wait for the green phrase All is Happy, you can start planning now!

Terminal window 4
* rosrun baxter_block_stack move_arm_vision.py
* rosrun baxter_block_stack open_cv_vision.py

Terminal window 5
* rosrun baxter_block_stack move_arm_pick.py
* rosrun baxter_block_stack move_arm_place.py
