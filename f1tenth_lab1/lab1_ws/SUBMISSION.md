# Lab 1: Intro to ROS 2

## Written Questions

### Q1: During this assignment, you've probably ran these two following commands at some point: ```source /opt/ros/foxy/setup.bash``` and ```source install/local_setup.bash```. Functionally what is the difference between the two?

Answer: 

The first will allow you to use ROS2 specific commands like ```ros2 topic list```, while the second will allow you to 
use all of the code you assembled within your workspace while using ROS2.


### Q2: What does the ```queue_size``` argument control when creating a subscriber or a publisher? How does different ```queue_size``` affect how messages are handled?

Answer:

```
This is the size of the outgoing message queue. If you are publishing faster than roscpp can send the messages over the wire, roscpp will start dropping old messages. A value of 0 here means an infinite queue, which can be dangerous. See the rospy documentation on choosing a good queue_size for more information...This is the incoming message queue size roscpp will use for your callback. If messages are arriving too fast and you are unable to keep up, roscpp will start throwing away messages. A value of 0 here means an infinite queue, which can be dangerous.
```
*Source: https://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers*

Messages are placed in a queue on the outgoing and the incoming paths (publisher and subscriber respectively) as an attempt
to preserve some as much of the signal stream as possible. For example, if you are publishing at 10x the rate you are subscribing, you 
will start to fill the queue on the subscribing end and you would only use the most recent message from the subscriber before you spin. 


### Q3: Do you have to call ```colcon build``` again after you've changed a launch file in your package? (Hint: consider two cases: calling ```ros2 launch``` in the directory where the launch file is, and calling it when the launch file is installed with the package.)

Answer: 

For direct usage from the launch directory, no rebuild is needed. For usage post-installation, a rebuild is required to reflect the changes.
