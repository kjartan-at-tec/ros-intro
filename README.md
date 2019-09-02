# Hands-on introduction to ROS
## Requirements
- Working installation of ROS.
- Turtlesim package installed. Otherwise, do `sudo apt-get install ros-kinetic-turtlesim`
## Activities step-by-step
### Check your ROS environment
#### Steps
1. Open a terminal. Execute `roscore`.
2. From the information given at startup, make a note of the following
   - rosdistro and rosversion
   - `ROS_MASTER_URI` This is the address other nodes need in order to communicate with the master, and hence with each other. 
### Get acquainted with Turtlesim
The Turtlesim package is a simple 2D simulation of mobile robots that move according to the unicycle model. It is excellent for getting an understanding of basic ROS concepts.
#### Concepts involved
- Distributed programming
- packages
- Master node (roscore) and other nodes
- Executing a node: `rosrun`
- Internode communication
  - *topics*
  - services
#### Steps
1. Start turtlesim. Open a new terminal window. Execute `rosrun turtlesim turtlesim_node`, which will run the executable `turtlesim_node` from the package `turtlesim`.
2. Find out how to interact with the turtlesim node.
   1. List topics: `rostopic list`. Get information about specific topics, for instance `rostopic  info /turtle1/cmd_vel`. What is the _type_ of the messages exchanged over this topic?
### Start
## Resources
- [http://wiki.ros.org/turtlesim]
- 
