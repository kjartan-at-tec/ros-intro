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
### Create a package to control the turtle
#### Concepts involved
- Catkin workspace
- ROS packages
- Building executable
- Set environment variables 
#### Steps
1. Create your [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace): `mkdir -P catkin_ws/src`, and `cd catkin_ws`, followed by `catkin_make`.
2. Create your first package. Go to the source folder `cd ~/catkin_ws/src`. Then execute `catkin_create_pkg turtle_controller roscpp std_msgs geometry_msgs`. This will create your package folder. 
3. Add some code to your package. Go to the package source folder `cd ~/catkin_ws/src/turtle_controller/src`. Then get the example cpp file from this repository `wget https://github.com/kjartan-at-tec/ros-intro/blob/master/print_pose.cpp`
4. Compile the code and run your node.
   - Make modifications to your package's `CMakeLists.txt`
   - Go to the catkin workspace root `cd ~/catkin_ws/`
   - Compile `catkin_make`
   - Set environment variables, so that ROS can find your package `source /devel/setup.bash`
   - Run your node `rosrun turtle_controller turtle_controller_print_pose`
5. Modify the code to do open-loop control of the turtle, making it move in a circle of a given radius.
### Programming challenge 
Implement a feedback controller with proportional gain that will make the turtle move to a desired point in the plane. Some hints
- Include math.m to get access to  `atan2` function: `#include "math.h"`
- If the variables `dx` and `dy` hold the distance to the goal point in the two directions, then the angle to the goal can be calculated with `double goal_dir = atan2(dy,dx);`
- Make the linear velocity proportional to the distance to the goal, with a proportional gain K. Make the angular velocity be proportional to the error in heading with a gain that is larger than K.   

## Resources
- [http://wiki.ros.org/turtlesim]
- 
