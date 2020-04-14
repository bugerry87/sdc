#!/usr/bin/env python3

#ros
import rospy
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import PoseWithCovarianceStamped as Pose, Point, Vector3
from visualization_msgs.msg import Marker


class TrackViz:
    def __init__(self, name='TrackViz', topic='~/odom', frame_id='map', skip=0):
        '''
        Initialize an TrackViz node.
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
        self.color = ColorRGBA(0.0,1.0,0.0,1.0)
        self.scale = Vector3(0.1,0.0,0.0)
        self.pub = rospy.Publisher('{}/markers'.format(self.name), Marker, queue_size=1000)
        self.reset()
        
        ## init the node
        rospy.Subscriber(self.topic, Pose, self.__update__)
    
    def reset(self):
        '''
        Resets Markers.
        '''
        self.input_counter = 0
        self.old_pose = Point(0.0,0.0,0.0)
        marker = Marker()
        marker.id = 0
        marker.action = marker.DELETEALL
        self.pub.publish(marker)

    def __update__(self, pos:Pose):
        '''
        Publishes a new marker based on the current state.
        
        Args:
            imu [Imu]:      The ROS message 'sensor_msgs.msg.Imu'.
            delta [float]:  Time delta between the timestamps of the messages.
        '''
        marker = Marker()
        marker.id = pos.header.seq
        marker.header.frame_id = self.frame_id
        marker.header.stamp = pos.header.stamp
        marker.header.seq = pos.header.seq
        marker.type = Marker.LINE_STRIP
        marker.action = marker.ADD

        marker.color = self.color
        marker.scale = self.scale
        marker.pose.orientation.w = 1.0

        marker.points = [
            self.old_pose,
            pos.pose.pose.position
        ]
        
        self.old_pose = pos.pose.pose.position
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
            default='TrackViz',
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
    rospy.init_node('TrackViz', anonymous=False)
    rospy.loginfo("Init node '{}' on topic '{}'".format(args.base_name, args.topic))
    imu = TrackViz(args.base_name, args.topic, args.frame_id, args.skip)
    rospy.loginfo("Node '{}' ready!".format(args.base_name))
    rospy.spin()

