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
  - [**topics**](http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics): A message board identified by its name. Any nodes can publish messages to a topic, and any nodes can subscribe to topics to receive messages that are published.
  - **services**: A node can provide a service, which other nodes call. 
#### Steps
1. Start turtlesim. Open a new terminal window. Execute `rosrun turtlesim turtlesim_node`, which will run the executable `turtlesim_node` from the package `turtlesim`.
2. Find out how to interact with the turtlesim node.
   1. Get info about **topics**. First  list available topics: `rostopic list`. Get information about specific topics, for instance `rostopic  info /turtle1/cmd_vel`, which gives info about the topic used by the specific turtle node `turtle1` to receive commands about velocity. What is the _type_ of the messages exchanged over this topic? You can find more info on specific message type using `rosmsg info <NameofMessageType>`.
   2. Get info about **services**. List available services: `rosservice list`. Note that some are specific for the turtle `turtle1`. To get info about a particular service, for instance the service that will spawn a new turle in the simulation: `rosservice info /spawn`. Note what arguments this service accepts. 
3. Move the turtle around. In order to do achieve this we need to publish messages to the topic `/turle1/cmd_vel`. Type `rostopic pub /turtle1/cmd_vel geometry_msgs/Twist` followed by a <tab>. You see that the message type `geometry_msgs/Twist` takes 6 arguments: Linear velocity in x, y, z directions, and rotational velocity in x, y, z. Since the turtle behaves like a unicycle, it will only take into account the values "linear: x" and "angular: z". In addition, for a single published message it receives, it will hold that velocity for one second before returning to zero velocity. 
   1. Move forward: `rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0, 0]' '[0, 0, 0]'` 
   2. Turn (approximately) 180 degrees: `rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, 3.14]'` 
4. To find the pose (position and orientation) of the turtle: `rostopic echo /turtle1/pose`. The origin is in the lower left corner, with  the x-axis pointing to the right and the y-axis pointing upwards. 
#### Programming challenge
Spawn a new turtle in the middle of the screen. Make it move in outgoing spiral.
### 

## Resources
- [http://wiki.ros.org/turtlesim]
- 
