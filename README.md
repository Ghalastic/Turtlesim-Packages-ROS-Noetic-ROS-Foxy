# Turtlesim Packages For ROS Noetic & ROS Foxy
####
## Task Description:-
#### 
Manipulate (experiment) with turtlesim packages in both ROS Noetic and ROS Foxy.
#### 
## Turtlesim in ROS Noetic:-
#### 
## Getting Started with Turtlesim:- 
#### 
- Open a terminal and source the ROS Noetic setup script:
#### 
```bash
source /opt/ros/noetic/setup.bash
```
####
- Start the roscore:
####
```bash
roscore
```
#### 
![roscore-code](https://github.com/user-attachments/assets/6ceaa81d-fe97-40c3-a15d-22f5c535445a)
#### 
- install and start the turtlesim:
####
```bash
sudo apt-get install ros-$(rosversion -d)-turtlesim
```
####
### Drawing an Arc, and Circle:
#### 
- Open another terminal, source the script, and start the roscore:
####
```bash
source /opt/ros/noetic/setup.bash
roscore
```
#### 
![roscore-code](https://github.com/user-attachments/assets/6ceaa81d-fe97-40c3-a15d-22f5c535445a)
####  
- Open another terminal, source the script, and run turtlesim:
####
```bash
source /opt/ros/noetic/setup.bash
rosrun turtlesim turtlesim_node
```
#### 
![rosrun](https://github.com/user-attachments/assets/c62bd4eb-3bc5-46fd-85f8-b228dc45c9c8)
####
- Open another terminal, source the script, and run this command:
#### 
```bash
source /opt/ros/noetic/setup.bash
rosrun turtlesim turtle_teleop_key
```
#### 
![teleop](https://github.com/user-attachments/assets/c0233caf-8c32-494b-8ebd-364fec392282)
#### 
### The Concept of Publishing and Subscribing:
- Publisher: turtle teleop key node
- Subscriber: turtlesim node
#### 
#### They communicate with each other over a rostopic with the turtle teleop keynote node publishing the keystrokes on a topic and the turtlesim node subscribing to the same topic to receive the keystrokes
#### 
#### To visualize their relationship, we can use the rqt graph:
- Open another terminal, source the script, and run this command:
####
```bash
source /opt/ros/noetic/setup.bash
rosrun rqt_graph rqt_graph
```
#### 
![rqtcoce](https://github.com/user-attachments/assets/ad9cd5f6-8b3a-45aa-8d7d-ea9d67293f39)
#### 
- The output should be this graph on a new terminal:
#### 
![rqt graph](https://github.com/user-attachments/assets/e0856ffb-4e26-4128-b1a6-f3be2fa3b39b)
#### 
#### Graph Explanation:
####
The graph shows:
####
- Publisher: /teleop_turtle node
- Subscriber: /turtlesim node
#### 
Which also shows that they're communicating over a rostopic which is: /turtle1/cmd_vel, they communicate with each other by sending and receiving the same type of message.
#### 
#### A topic type is defined by the message type published on it, the type of message sent on a topic can be determined using the following commands:-
#### 
- Open another terminal, source the script, and run this command:
#### 
```bash
source /opt/ros/noetic/setup.bash
rostopic type /turtle1/cmd_vel
```
#### 
![rostopic](https://github.com/user-attachments/assets/7b4462ee-fa89-46aa-be16-8e57958f8bf2)
#### 
- And to find the arguments for this message type you must open another terminal, source the script, and run this command:
####
```bash
source /opt/ros/noetic/setup.bash
rosmsg show geometry_msgs/Twist
```
####
- the output should be the 3 linear components and the 3 angular components needed to define this message type:
####
![geom](https://github.com/user-attachments/assets/5aab369e-ff36-4775-8c9c-4bade0550651)
#### 
- Now we have all 3 inputs (topic type, message type, message arguments) to run the rostopic publisher command on a new terminal:
####
```bash
source /opt/ros/noetic/setup.bash
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, -1.8]'
```
#### 
![pub -1](https://github.com/user-attachments/assets/266e0a5d-3ba0-4ebb-9dd5-d15807d2bfc1)
####
And we should get this arc drawing:
#### 
![arcturtle](https://github.com/user-attachments/assets/316ef956-3585-4bb1-b680-01929bc8c9b6)
#### 
- To draw a circle, we must modify the code we wrote on the last terminal we opened by first opening a new terminal, sourcing the script and then deleting the first drawing, after that modify the last command line on the previous terminal to become like this:
####
```bash
rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, -1.8]'
```
#### 
![pub](https://github.com/user-attachments/assets/c221ff58-8da9-40da-83ec-49b9568da8ce)
#### 
![turtlecirclecute](https://github.com/user-attachments/assets/1a74a213-0cd7-41ff-a50b-1c9561ec1f79)
#### 

