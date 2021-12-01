# Robotis-OP3-MPC-walking
MPC based gait generation and kinematic control of Robotis OP3 humanoid

**SET UP GUIDE**

1) Install ROS Kinetic or [Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu). It probably works also on Noetic but it has not been tested there yet.
2) Install ROS packages for [ROBOTIS OP3](https://emanual.robotis.com/docs/en/platform/op3/recovery/#installing-robotis-ros-packages).
3) Navigate into the catkin workspace of the robotis packages and clone this repository inside the directory */src/ROBOTIS-OP3*.
4) Navigate into the directory */src/ROBOTIS-OP3/Robotis-OP3-MPC-walking/mpc_walking_demo* and clone there the [blasfeo](https://github.com/giaf/blasfeo) and [hpipm](https://github.com/giaf/hpipm) repositories. Alternatively, you can unzip the file blasfeo&hpipm.zip (which is inside */src/ROBOTIS-OP3/Robotis-OP3-MPC-walking/mpc_walking_demo*) and place the *blasfeo* and *hpipm* directory inside the */mpc_walking_demo* directory.
5) Copy and paste the file */src/ROBOTIS-OP3/Robotis-OP3-MPC-walking/mpc_walking_demo/robotis_op3.urdf* inside */src/ROBOTIS-OP3-Common/op3_description/urdf*.
6) Navigate to the file */src/ROBOTIS-OP3/op3_kinematics_dynamics/include/op3_kinematics_dynamics/op3_kinematics_dynamics_define.h* and modify (around line 29)
```
#define MAX_ITER
```
to
```
#define MAX_ITER_
```
6) Compile the packages (inside the catkin workspace)
```
catkin_make DCMAKE_BUILD_TYPE=Release
```
7) In order to run the gazebo simulation, open a new terminal and type (assuming you have not sourced yet the devel/setup.bash)
```
source devel/setup.bash
roslaunch op3_gazebo robotis_world.launch
```
then from another terminal 
```
source devel/setup.bash
roslaunch diag_op3_manager diag_op3_gazebo.launch.
```
**CODE GUIDE**


