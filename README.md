# Robotis-OP3-MPC-walking
MPC based gait generation and kinematic control for the Robotis OP3 humanoid

**SET UP GUIDE**

1) Install ROS Kinetic or [Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu). It probably works also on Noetic but it has not been tested there yet.
2) Install the [ROBOTIS OP3](https://emanual.robotis.com/docs/en/platform/op3/recovery/#installing-robotis-ros-packages) ROS packages and all the required dependencies (starting from **1.1.2.4. Installing additional applications for ROBOTIS ROS Package**). If using ROS Melodic you may also need to: manually clone the repository [humanoid_msgs](https://github.com/ahornung/humanoid_msgs) into the */src* directory, install sbpl (type from terminal *sudo apt-get install ros-melodic-sbpl*), install ros control (type from terminal *sudo apt-get install ros-melodic-ros-control*, *apt-get install ros-kinetic-ros-controllers*, *apt-get install ros-kinetic-gazebo-ros-control*).
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
7) Compile the packages (type from terminal inside the catkin workspace)
```
catkin_make DCMAKE_BUILD_TYPE=Release
```
8) In order to run the gazebo simulation, open a new terminal and type (assuming you have not sourced yet the devel/setup.bash)
```
source devel/setup.bash
roslaunch op3_gazebo robotis_world.launch
```
then from another terminal 
```
source devel/setup.bash
roslaunch diag_op3_manager diag_op3_gazebo.launch
```
**CODE GUIDE**

1) This repository contains a robot manager node called *diag_op3_manager* and a motion module called *mpc_walking_demo*. 
2) The manager is in charge of applying the ROBOTIS Framework to the simulated or real hardware robot. Further details can be found [here](https://emanual.robotis.com/docs/en/software/robotis_framework_packages/tutorials/#creating-new-motion-module). In practice, the manager configures the robot and adds sensor and motion modules to the robotis controller. The system architecture is: <img src="https://emanual.robotis.com/assets/images/platform/op3/op3_027.png" width="300" height="270">
3) The *mpc_walking_demo* motion module is where the kinematic controller and the real time MPC gait generation are implemented. At each sampling time the method *op3LocomotionModule_walking_demo::process(..)* is executed, accessing the motor encoders for measurements and providing reference commands to the low level motor controllers. Motor readings and command are provided in real time. From the motion module it is also possible to subscribe to topics/services/actions which are not part of the robotis framework, e.g., the IMU sensor, camera stream. A description of the control architecture is avaialble in the *Robotis-OP3-MPC-walking.pdf* file. 
4) The MPC gait generation is realized via a simple implementation of the method proposed in:
*Scianca, N., De Simone, D., Lanari, L., & Oriolo, G. (2020). MPC for humanoid gait generation: Stability and feasibility. IEEE Transactions on Robotics, 36(4), 1171-1188*. 
