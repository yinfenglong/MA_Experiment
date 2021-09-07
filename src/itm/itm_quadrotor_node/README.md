# itm_quadrotor_node

## itm_quadrotot_control

## SITL

Install ROS Kinetic according to the [documentation](http://wiki.ros.org/kinetic/Installation), then [create a Catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

directory DataSSD/YanLI/Yan_ROS_ws

Terminal 1:
to connect to localhost, use this URL:

```
$roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

Terminal 2:
to run SITL wrapped in ROS the ROS environment needs to be updated, then launch as usual:
```
$cd <Firmware_clone>
$DONT_RUN=1 make px4_sitl_default gazebo
$source ~/catkin_ws/devel/setup.bash    # (optional)
$source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
$export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
$export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
$no_sim=1 make px4_sitl_default gazebo
```
Terminal 3:

```
$cd <Firmware_clone>
$source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
$roslaunch gazebo_ros empty_world.launch world_name:=$(pwd)/Tools/sitl_gazebo/worlds/iris.world
```
Terminal 4:
```
$roslaunch itm_nonlinear_mpc itm_nonlinear_mpc_sim_sitl.launch
```
Terminal 5: publish set point position topic

```
$rostopic pub /itm_quadrotot_control/set_point_pos geometry_msgs/PoseStamped“header:
```
Terminal 6:
```
roslaunch itm_nonlinear_mpc itm_nonlinear_mpc_sim_offboard.launch
```
