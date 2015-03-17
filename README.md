# baxter_block_stacking
Goals of this project are:
  * locate blocks using opencv
  * pick up block and move to block tower location using MoveIt
  * make use of collision objects to avoid knocking blocks over during movement
  
Instruction for running files (will eventually be turned into a launch file):

Terminal window 1
* sudo service network-manager stop
* sudo avahi-autoipd eth0

For terminal Window 2 and onwards please do these steps first
*cd baxter_ws/ (new tabs created after this step should already be in the correct folder)
*./baxter.sh

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
