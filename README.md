# ivaEdy
edy ros-related packages

## Usage
go to `~/YOUR/ROS_WORKING_SPACE`
```
cd src/
git clone https://github.com/ivaROS/ivaEdy.git
cd ..
catkin_make
```
Note: you may see error when generating cmakelist, please install any missing packages according to the error messages. 

## Run edy in the real world
Source the pacakges
```
source devel/setup.bash
```
First, launch the node of controller manager which will manage all loaded controllers
```
roslaunch edy_dualarm_control controller_manager.launch
```
If you get the problem of permission denied, try to do `sudo chmod 666 /dev/ttyUSB0`

Then, load joint position controllers and action controllers seperately to avoid synchrinization problem since action controller
has the dependency on its own joint position controllers.
```
roslaunch edy_dualarm_control start_joint_position_controller.launch
roslaunch edy_dualarm_control start_trajectory_action_controller_l.launch
roslaunch edy_dualarm_control start_trajectory_action_controller_r.launch
```

After starting all controllers, you need to publish `tf` information of robot
```
roslaunch edy_dualarm_description robot_state_pub.launch
```

The rest things you need to do is about setup for moveit,
```
roslaunch edy_dualarm_moveit_config move_group.launch
roslaunch edy_dualarm_moveit_config moveit_rviz.launch
```

Right now you should be good to go, you can launch any file from edy_dualarm_experiment or write your own code to let edy do 
something
For example:
```
roslaunch edy_dualarm_experiment pick.launch
