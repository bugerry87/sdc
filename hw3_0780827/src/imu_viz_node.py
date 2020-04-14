#!/usr/bin/env python3

#installed
import numpy as np

#ros
import rospy
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker


class IMUviz:
    GRAVITY = np.array([0,0,-9.80665])

    def __init__(self, name='IMUviz', topic='~/imu/data', frame_id='map', skip=0):
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
        self.skip = skip
        self.color = ColorRGBA(0.0,0.5,1.0,1.0)
        self.scale = Vector3(0.1,0.0,0.0)
        self.pub = rospy.Publisher('{}/markers'.format(self.name), Marker, queue_size=1000)
        self.reset()
        
        ## init the node
        rospy.Subscriber(self.topic, Imu, self.__update__)
    
    def reset(self):
        '''
        Resets the assumed IMU state.
        '''
        self.s = np.array([0.0,0.0,0.0])
        self.v = np.array([0.0,0.0,0.0])
        self.time = 0.0
        self.C = np.eye(3)
        self.input_counter = 0
        marker = Marker()
        marker.id = 0
        marker.action = marker.DELETEALL
        self.pub.publish(marker)

    def __update__(self, imu:Imu):
        '''
        Update routine, to be called by subscribers.
        
        Args:
            imu [Imu]: The ROS message 'sensor_msgs.msg.Imu'.
        '''
        ## update timestamp
        sec = imu.header.stamp.secs
        nsec = imu.header.stamp.nsecs * 1e-9
        time = sec + nsec
        self.input_counter += 1
        ## extract data
        a = imu.linear_acceleration
        w = imu.angular_velocity

        ## check valid condition
        if self.time == 0.0 or self.input_counter < self.skip:
            IMUviz.GRAVITY = np.array([a.x, a.y, a.z])
            self.time = time
            return
        
        delta = time - self.time
        if delta < 0.0:
            rospy.logwarn("Jump back in time detected reset: {}".format(rospy.get_caller_id()))
            self.reset()
            return
        self.time = time
        
        ## convert data to numpy
        a = np.array([a.x, a.y, a.z]) #.reshape(1,3)
        w = np.array([w.x, w.y, w.z]) * delta
        d = np.linalg.norm(w, ord=1) #L1 or L2??? L1 looks more close to reference!
                
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
        self.v += (self.a - IMUviz.GRAVITY) * delta
        self.s += self.v * delta
        
        ##publish the marker
        #rospy.loginfo("{} IMU:\n{}".format(rospy.get_caller_id(), imu))
        self.pub_marker(imu, delta)

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

        marker.color = self.color
        marker.scale = self.scale
        marker.pose.position = Point(*self.s)
        marker.pose.orientation.w = 1.0

        marker.points = [
            Point(*(-self.v * delta)),
            Point(0.0,0.0,0.0)
        ]
        
        #rospy.loginfo("{} Marker:\n{}".format(rospy.get_caller_id(), marker))
        self.pub.publish(marker)
        
 
if __name__ == '__main__':
    from argparse import ArgumentParser
    
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
        
        parser.add_argument(
            '--skip', '-s',
            metavar='INT',
            type=int,
            default=0,
            help='Number of frames to be skipped.'
            )
        
        return parser


    args, _ = init_argparse().parse_known_args()
    rospy.init_node('IMUviz', anonymous=False)
    rospy.loginfo("Init node '{}' on topic '{}'".format(args.base_name, args.topic))
    imu = IMUviz(args.base_name, args.topic, args.frame_id, args.skip)
    rospy.loginfo("Node '{}' ready!".format(args.base_name))
    rospy.spin()

