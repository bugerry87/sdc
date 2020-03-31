#!/usr/bin/env python3

#build in
import sys

#installed
import numpy as np

#ros
import rospy
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point, Vector3, Quaternion, Pose
from visualization_msgs.msg import Marker


class IMUviz:
    GRAVITY = np.array([0,0,9.80665])

    def __init__(self, name='IMUviz', topic='~/imu/data', frame_id='map'):
        '''
        Initialize an IMUviz node.
        Subscribes imu data.
        Publishes markers.
        
        Args:
            name [str]:     Base name of the node.
            topic [str]:    The topic to be subscribed.
            frame_id [str]: The frame_id in rviz where the markers get plotted at.
        '''
        self.name = name
        self.topic = topic
        self.frame_id = frame_id
        self.color = np.array([0.0,0.5,1.0,1.0])
        self.scale = np.array([0.1,0.0,0.0])
        self.reset()
        
        ## init the node
        self.pub = rospy.Publisher('{}/markers'.format(self.name), Marker, queue_size=1000)
        rospy.Subscriber(self.topic, Imu, self.__update__)
    
    def reset(self):
        '''
        Resets the assumed IMU state.
        '''
        self.s = np.array([0.0,0.0,0.0])
        self.v = np.array([0.0,0.0,0.0])
        self.time = 0.0
        self.C = np.eye(3)
        self.history = []

    def __update__(self, imu):
        '''
        Update routine, to be called by subscribers.
        
        Args:
            imu [Imu]: The ROS message 'sensor_msgs.msg.Imu'.
        '''
        ## update timestamp
        sec = imu.header.stamp.secs
        nsec = imu.header.stamp.nsecs * 1e-9
        time = sec + nsec
        ## extract data
        a = imu.linear_acceleration
        w = imu.angular_velocity

        ## check valid condition
        if self.time == 0.0:
            self.time = time
            return
        
        delta = time - self.time
        self.time = time
        
        ## convert data to numpy
        a = np.array([a.x, a.y, a.z]) #.reshape(1,3)
        w = np.array([w.x, w.y, w.z]) * delta
        d = w.sum()
        
        ## process Oliver J. Woodman's algo.
        B = np.array([
            0, -w[2], w[1],
            w[2], 0, -w[0],
            -w[1], w[0], 0
        ]).reshape(3,3)

        Bsin = np.sin(d)/d * B
        Bcos = (1-np.cos(d))/(d**2) * (B**2)
        self.C = np.dot(self.C, np.eye(3) + Bsin + Bcos)
        self.a = np.dot(self.C, a)
        self.v += (IMUviz.GRAVITY - self.a) * delta
        self.s += self.v * delta

        o = imu.orientation #just for reference
        self.gyro = np.array([o.x, o.y, o.z, o.w])
        
        ##publish the marker
        #rospy.loginfo("{} IMU:\n{}".format(rospy.get_caller_id(), imu))
        self.pub_marker(imu, delta)
        self.history.append((self.time, *self.v))

    def pub_marker(self, imu, delta):
        '''
        Publishes a new marker based on the current state.
        
        Args:
            imu [Imu]:      The ROS message 'sensor_msgs.msg.Imu'.
            delta [float]:  Time delta between the timestamps of the messages.
        '''
        marker = Marker()
        marker.id = imu.header.seq
        marker.header.frame_id = self.frame_id
        marker.header.stamp = imu.header.stamp
        marker.header.seq = imu.header.seq
        marker.type = Marker.LINE_STRIP
        marker.action = marker.ADD

        marker.color = ColorRGBA(*self.color)
        marker.scale = Vector3(*self.scale)
        marker.pose.position = Vector3(*self.s)
        marker.pose.orientation.w = 1.0

        marker.points = [
            Point(*(-self.v * delta)),
            Point(0.0,0.0,0.0)
        ]
        
        #rospy.loginfo("{} Marker:\n{}".format(rospy.get_caller_id(), marker))
        self.pub.publish(marker)
        
 
if __name__ == '__main__':
    from argparse import ArgumentParser
    import matplotlib.pyplot as plt
    
    def init_argparse(parents=[]):
        ''' init_argparse(parents=[]) -> parser
        Initialize an ArgumentParser for this module.
        
        Args:
            parents: A list of ArgumentParsers of other scripts, if there are any.
            
        Returns:
            parser: The ArgumentParsers.
        '''
        parser = ArgumentParser(
            description="ROS node to visualize IMU data.",
            parents=parents
            )
        
        parser.add_argument(
            '--base_name', '-n',
            metavar='STRING',
            default='IMUviz',
            help='Base name of the node.'
            )
        
        parser.add_argument(
            '--topic', '-t',
            metavar='TOPIC',
            default='~/imu/data',
            help='The topic to be subscribed.'
            )
        
        parser.add_argument(
            '--frame_id', '-f',
            metavar='STRING',
            default='map',
            help='The frame_id for rviz to plot the markers at.'
            )
        
        return parser


    args, _ = init_argparse().parse_known_args()
    rospy.init_node(args.base_name, anonymous=False)
    rospy.loginfo("Init node '{}' on topic '{}'".format(args.base_name, args.topic))
    imu = IMUviz(args.base_name, args.topic, args.frame_id)
    rospy.loginfo("Node '{}' ready!".format(args.base_name))
    
    while True:
        inp = input("\n".join((
            "Press 'r' to reset IMU state,",
            "  or 'p' for a matplot,",
            "  or 'q' for quit (q): ")))
        if inp == 'r':
            imu.reset()
            rospy.loginfo("Reset node: '{}'!".format(args.base_name))
        elif inp == 'p':
            data = np.array(imu.history)
            if data.size:
                plt.plot(data[:,0], data[:,1:])
                plt.title("IMU body velocity")
                plt.legend(("x", "y", "z"))
                plt.show()
            else:
                print("\nWARNING: Yet no data available!\n")
        else:
            break
