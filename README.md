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

## Conclusion
This lab exercise provided hands-on experience with ROS nodes, message passing, and basic publisher-subscriber communication. By implementing and testing `talker.py` and `listener.py`, we gained a deeper understanding of how ROS manages inter-process communication and real-time data exchange between nodes.

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

