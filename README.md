# IMUviz

A ROS node for tracking and visualizing IMU data based on Oliver J. Woodman's "Inertial Navigation"

HW3_0780827 Gerald Baulig

GitHub [https://github.com/bugerry87/sdc.git]

## Enironment

Tested on:

- Ubuntu 18.04.4 LTS + ROS Melodic 1.14.5 + Python 3.6.9
- Ubuntu 16.04 LTS + ROS Kinetic 1.12.7 + Python 3.5.2

## Requirements

**apt-get**

- python3-tk

**pip3**

- rospy
- rospkg
- numpy
- matplotlib

## Installation

1. Clone this package into to `src` of your catkin workspace.

2. Ensure that all ROS environment variables are set. i.e.:

```
$ source /opt/ros/melodic/setup.sh
$ source ~/catkin_ws/devel/setup.sh
```

3. Ensure the ROS-Core is running with i.e.: `$ roscore &`

4. Navigate to your catkin workspace i.e.: `$ cd ~/catkin_ws`

5. Make the package i.e.: `$ catkin_make --pkg hw3_0780827`

Done!

## Quick Run

For a simple run make sure the file `imu_viz_node.py` is executable.
Then type:

```
$ rosrun hw3_0780827 imu_viz_node.py
```

and the following output should appeare:

```
[INFO] [1585567197.469376]: Init node 'IMUviz' on topic '~/imu/data'
[INFO] [1585567197.472197]: Node 'IMUviz' ready!
Press 'r' to reset IMU state,
  or 'p' for a matplot,
  or 'q' for quit (q):
```

Now the node subscribes the mantioned topic and is ready to receive data.
On the other hand, the node published `Marker` messages for each `IMU` message.
Subscribe the topic `IMUviz/markers` i.e. via rviz and run a rosbag containing IMU messages i.e.:

```
$ rviz &
$ rosbag play -r 0.5 ~/Desktop/sdc_hw3.bag
```

The node is representing one IMU-sensor.

- You can reset the node internal state back to zero by typing `r`.
- Plot the history of velocities by typing `p`.
- Or terminate/quit the node by typing `q` (default).

## Further Controles

Type: `$ rosrun hw3_0780827 imu_viz_node.py -h` to obtain a help message with details about the given start parameters:

```
usage: imu_viz_node.py [-h] [--base_name STRING] [--topic TOPIC]
                       [--frame_id STRING]

ROS node to visualize IMU data.

optional arguments:
  -h, --help            show this help message and exit
  --base_name STRING, -n STRING
                        Base name of the node.
  --topic TOPIC, -t TOPIC
                        The topic to be subscribed.
  --frame_id STRING, -f STRING
                        The frame_id for rviz to plot the markers at.
```

Thus, you have full control of the node name, which topic should be subscribed and to which frame the markers should be plotted at.

