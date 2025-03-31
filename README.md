# Laboratory 2 Report

## Overview
In this lab exercise, we developed multiple programs to enhance our understanding of ROS (Robot Operating System) and Python programming. The objective was to create, compile, and execute ROS nodes while analyzing their functionality.

## General Considerations
All the code for this lab was written in Python and executed using ROS. To test the programs, both the original script and its corresponding launch file were created. The following command was used in the terminal to run the programs:

```bash
$ roslaunch Practicas_Lab <launch file>
```

## Installation and Setup
### Prerequisites
Ensure you have the following installed:
- ROS (Robot Operating System)
- Python 2.7 or 3.x
- `rospy`, `roscpp`, and `std_msgs` packages

### Cloning the Repository
To clone this repository, run the following command:
```bash
$ git clone https://github.com/your-username/Lab2-ROS.git
$ cd Lab2-ROS
```

### Building the Package
After cloning, navigate to your ROS workspace and build the package:
```bash
$ cd ~/catkin_ws/src
$ ln -s ~/Lab2-ROS Practicas_Lab
$ cd ~/catkin_ws
$ catkin_make
$ source devel/setup.bash
```

## Lab 2: Basic ROS Nodes
In this exercise, we worked with two Python files: `listener.py` and `talker.py`. The tasks included:

- Creating a ROS package named `Practicas_Lab` with dependencies on `rospy`, `roscpp`, and `std_msgs`.
- Placing `listener.py` and `talker.py` in the package.
- Compiling the package.
- Running the `talker` node.
- Running the `listener` node.
- Analyzing and documenting their functionality.

### `talker.py`
The `talker.py` script publishes messages to a topic named `chatter`. Below is the complete code:

```python
#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10 Hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
```

### `listener.py`
The `listener.py` script subscribes to the `chatter` topic and logs received messages. Below is the full implementation:

```python
#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.data)
    
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chatter", String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
```

## Running the Nodes
To run the nodes, execute the following commands in separate terminals:

**Start the `talker` node:**
```bash
$ rosrun Practicas_Lab talker.py
```

**Start the `listener` node:**
```bash
$ rosrun Practicas_Lab listener.py
```
### Publisher and Subscriber

This code is also known as a "publisher and subscriber" model. The ROS talker node starts with a shebang declaration to ensure the script runs as a Python script within the ROS environment. It then imports rospy, which is necessary for creating ROS nodes, and std_msgs.msg.String to enable the use of the standard String message type for communication.

Next, the script defines a publisher that sends messages to the "chatter" topic using the String message type. The queue_size parameter is included to manage message storage in case subscribers are slow to process them. The node is then initialized with the name talker, and the anonymous=True flag ensures it gets a unique name, preventing conflicts when multiple instances run simultaneously.

To regulate the message transmission rate, a rate object is created to maintain a loop frequency of 10 Hz. The main loop continuously executes until the node is shut down. During each iteration, a "hello world" message is generated, including a timestamp, and logged for debugging. The message is then published to the "chatter" topic, and the loop pauses momentarily to maintain the desired frequency.

For debugging and monitoring, rospy.loginfo() is used, allowing messages to be printed to the console, logged in a file, and sent to rosout, making it easier to track node activity. Finally, the script includes exception handling to catch rospy.ROSInterruptException, which prevents errors when the node is stopped using Ctrl+C or any other shutdown command.

With the talker node in place, the next step is to implement a listener node to receive and process the published messages. The listener.py node in ROS subscribes to the "chatter" topic, which transmits messages of type std_msgs.msg.String. When a new message is received, a callback function is triggered to process it.

To ensure each node has a unique name, the anonymous=True argument is added to rospy.init_node(), preventing conflicts if multiple listeners run simultaneously.

Finally, rospy.spin() is used to keep the node running until it is manually stopped, ensuring it continuously listens for incoming messages.

## Lab2 Medium
For this part of the lab, we were tasked with implementing the following:
* Create a keyboard control for turtlesim.
* Draw a square and an equilateral triangle using turtlesim (without a controller).

To enable keyboard input, we implemented the following function to read and process the keyboard events:
```python
def get_key():
    """Lee una tecla del teclado sin necesidad de presionar Enter."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)  # Lee un solo caracter
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key
```
Obtaining the next results:

Using WASD:  
<img width="322" alt="libre" src="https://github.com/user-attachments/assets/e22717bc-f985-444e-8a45-1d243561caa6" />

Square  
<img width="317" alt="cuadrado" src="https://github.com/user-attachments/assets/9cda7364-c34d-4207-8cca-4fe1f94ed845" />

Triangle  
<img width="304" alt="triangulo" src="https://github.com/user-attachments/assets/c35fb0a8-82d2-41d7-b4dd-60e6c8dc7837" />

Lab 2: Advanced Controllers
For this final part of the lab, the instructions were as follows:

* Position control for turtlesim (P)

* Position control for turtlesim (PI)

* Position control for turtlesim (PID)

* Compare the performance of each controller using Plot Juggler or another plotting tool.

* Report the results in Markdown.

Proportional Control (P)
![image](https://github.com/user-attachments/assets/8625e354-4896-4642-9250-eae51025ea32)


Proportional integral control (PI)
![image](https://github.com/user-attachments/assets/d5cf23d1-5bf4-4f4a-acff-fc0532921049)


Proportional integral derivative control (PID)
![image](https://github.com/user-attachments/assets/21742f9c-6a32-48a7-b5b4-e5789b78cb80)


In Plot Juggler, when I compared the performance of the different controllers (P, PI, and PID), I observed the changes in the x and y positions, as well as the linear velocity, over time.

P-Control: The x and y positions gradually approached the target but never fully reached it due to steady-state error. The system hovered near the target, staying within a small margin of error. The linear velocity started high as the robot reacted to the position error but decreased as the robot got closer to the target, and it took a while to fully stop.

PI-Control: The x and y positions eventually reached the target, eliminating the steady-state error. However, it took slightly longer to converge compared to P-Control, and there was some overshoot, where the robot briefly passed the target before stabilizing. The linear velocity decreased over time as the integral term accumulated past errors, but the response was slower than in P-Control. Eventually, the system stabilized, correcting for steady-state error, but it took longer to settle than P-Control.

PID-Control: The x and y positions reached the target quickly and accurately, with minimal error. The linear velocity started high but decreased smoothly and quickly as the system corrected the error. The derivative term helped prevent overshoot, ensuring that the system stabilized quickly and accurately at the target position. The robot approached the target with minimal fluctuations in velocity, resulting in a smooth and efficient response.

Summary:
* P-Control: Persistent position errors, fluctuating velocities, and potential oscillations around the target.

* PI-Control: Eliminated steady-state error but overshot and took longer to stabilize compared to P-Control.

* PID-Control: Best performance, with quick and accurate convergence of positions and rapid stabilization of the linear velocity.

