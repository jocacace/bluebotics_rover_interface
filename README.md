# rover_ctrl_interface
This document describes how to setup, start and control the casy rover made by bluebotics. 

### Interfacing with the robot

To turn on the robot motors and the onboard pc you must pull up the black lever on robot base. On the same plaette you can find 3 leds: 
* Powe on: a blue light is on when the robot is powered
* OFF: the lever to pull to start the rover
* ANT not ok: this is not important
* Safety not ok: when the red light is on, the motors are disbled. To turn off the red light just push the safety mushroom

To control the robot motors you can use the industrial pc of the robot itself. To use this pc, you can both use the ethernet cable joining a private LAN network and the VGA cable along with usb mouse and keyboard. You can plug mouse and keyboard using the USB hub of the rover. In addition, the rover is equipped with a usb dongle wifi, to connect with other external networks. Be careful, when the robot starts usually the dongle is turned off. To activate it, you should unplung and plug by hand.

#### Connecting to the industrial pc

To connect your own laptop to the industrial pc of the robot you can use the gray ethernet cable. The ip address settings are shown in the following: 
**Rover**: The ip address of the rover is 192.168.0.21. You must properly configure your network to connect with the robot. In this context, your IP address must be manually configured to be of this type: 192.168.0.XX (where xx can be any numebr between 2 and 253, expect for 21, already assigned to the robot). To test that the communication works properly use the ping command in your terminal:
             
             $ ping 192.168.0.21
Note that if you are using a Linux based OS, you can save this IP address as a know host editing the file  /etc/hosts:

             $ sudo nano /etc/hosts
Adding the following line: 

             192.168.0.21    rover
Now you can ping the robot using the following command:

             $ ping rover
To execute commands on the robot you should login to the onboard pc usgin ssh protocol:

             $ ssh -l casy_rover rover 
If everything is working properly you should insert the casy_rover's user password.
You can also login to the remote host using your window manager. Usually, you window manager has an option like: connect to the server. After logged in with your windows manager you can also edit source files using your own code editor.

## Launch rover control nodes
To launch the rover control node you can use this ROS packages. First of all you should setup the *can* interfaces. To do this use the bash script **setup_can0.sh** contained in the root of this package:

             $ roscd rover_ctrl_interface
             $ sudo ./setup_can0.sh
             
Finally, you can launch the rover control node:

             $ roslaunch rover_ctrl_interface rover_ctrl_interface.launch

Now you can read the state of the robot and its sensors and control it using the following ROS topics:

* */cmd_vel*: this topic receives the velocity to move the base of the robot. This is a *geometry_msgs/Twist* message, in which you can specify the linear velocity of the base along the x direction, and the angular velocity along the z axis.
* */joy*: this topic is started with the rover control interface and receives the body velocity to move the robot. Be careful, to move the robot with autonomous control or directly with the joypad you should follow the **safety rules** explained below.
* */scan*: publishes the laser scanner data
* */raw_fix*: publishes the GPS data
* */imu/data*: published the orientation information of the robot calculated with the IMU sensor.

The structure of the robot is also published using the static transformation using the tf.launch launch filed placed in the launch directory of this node.

To use Rviz with onboard robot nodes (like the rover control interface), you should configure your on pc (with ROS already installed on). To do this you must export two environment variables:

             $ export ROS_MASTER_URI=http://192.168.0.21:11311
             $ export ROS_HOSTNAME=[YOUR IP ADDRESS]

From this moment you can receive the data from another (remote) *roscore*
# Safety rules
To move the robot you can choose the following ways:
* Using the joypad: to do this, you should press LB button. Later you can use the left stick to command the forward direction of the robot, and the right one to control the rotational movements
* Using an autonomous motion plan: in this case you should take pressed both LB and RB buttons
