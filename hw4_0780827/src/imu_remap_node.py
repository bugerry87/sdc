#!/usr/bin/env python3

#installed
import numpy as np
from scipy.spatial.transform import Rotation as R

#ros
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion


class IMUremap:

    def __init__(self, name='IMUremap', topic='~/imu/data', frame_id='map', target=(0,0,0,1)):
        '''
        Initialize an IMUremap node.
        Subscribes imu data.
        Publishes markers.
        
        Args:
            name [str]:     Base name of the node.
            topic [str]:    The topic to be subscribed.
            frame_id [str]: The frame_id in rviz where the markers get plotted at.
            target [tuple]: The remap quaternion.
        '''
        self.name = name
        self.topic = topic
        self.frame_id = frame_id
        self.target = R.from_quat(target)
        self.pub = rospy.Publisher('{}/imu_data'.format(self.name), Imu, queue_size=1000)
        
        ## init the node
        rospy.Subscriber(self.topic, Imu, self.__update__)

    def __update__(self, imu:Imu):
        '''
        Update routine, to be called by subscribers.
        
        Args:
            imu [Imu]: The ROS message 'sensor_msgs.msg.Imu'.
        '''
        o = imu.orientation
        source = R.from_quat([o.x, o.y, o.z, o.w])
        remapped = np.matmul(self.target.as_matrix(), source.as_matrix())
        imu.orientation = Quaternion(*R.from_matrix(remapped).as_quat())
        self.pub.publish(imu)
        
 
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
            default='IMUremap',
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
            '--target', '-q',
            metavar='FLOAT',
            type=float,
            nargs=4,
            default=(0,0,0,0),
            help='The target quaternion.'
            )
        
        return parser


    args, _ = init_argparse().parse_known_args()
    rospy.init_node('IMUremap', anonymous=False)
    rospy.loginfo("Init node '{}' on topic '{}'".format(args.base_name, args.topic))
    imu = IMUremap(args.base_name, args.topic, args.frame_id, args.target)
    rospy.loginfo("Node '{}' ready!".format(args.base_name))
    rospy.spin()

