#!/usr/bin/env python

'''
This script makes Gazebo less fail by translating gazebo status messages to odometry data.
Since Gazebo also publishes data faster than normal odom data, this script caps the update to 20hz.
Winter Guerra
'''

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Transform, TransformStamped
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Header
import numpy as np
import math
import tf2_ros
import random
import tf

class OdometryNode:
    # Set publishers
    pub_odom = rospy.Publisher('/odom', Odometry, queue_size=1)

    def __init__(self):
        # init internals
        self.last_received_pose = Pose()
        self.last_received_twist = Twist()
        self.last_recieved_stamp = None
        self.last_x = None
        self.last_y = None
        self.last_pos = None

        # Set the update rate
        rospy.Timer(rospy.Duration(.05), self.timer_callback) # 20hz

        self.tf_pub = tf2_ros.TransformBroadcaster()

        # Set subscribers
        rospy.Subscriber('/gazebo/link_states', LinkStates, self.sub_robot_pose_update)

    def sub_robot_pose_update(self, msg):
        # Find the index of the racecar
        try:
            arrayIndex = msg.name.index('selfie::base_link')
        except ValueError as e:
            # Wait for Gazebo to startup
            pass
        else:
            # Extract our current position information
            self.last_received_pose = msg.pose[arrayIndex]
            self.last_received_twist = msg.twist[arrayIndex]
        self.last_recieved_stamp = rospy.Time.now()

    def timer_callback(self, event):
        if self.last_recieved_stamp is None:
            return

        cmd = Odometry()
        cmd.header.stamp = self.last_recieved_stamp
        cmd.header.frame_id = 'odom'
        cmd.child_frame_id = 'base_link'
        cmd.pose.pose = self.last_received_pose
        cmd.twist.twist = self.last_received_twist
        self.pub_odom.publish(cmd)

        if self.last_x is None:
            self.last_x = cmd.pose.pose.position.x
            self.last_y = cmd.pose.pose.position.y
            self.last_pos = cmd.pose.pose.position


        p = Pose()
        p.position.x = cmd.pose.pose.position.x
        p.position.y = cmd.pose.pose.position.y
        p.position.z = cmd.pose.pose.position.z
        # p.x += random.uniform(-0.2,0.2)
        # p.y += random.uniform(-0.2,0.2)
        dist = math.sqrt((self.last_pos.x-p.position.x)**2 + (self.last_pos.y-p.position.y)**2)
        self.last_pos.x = cmd.pose.pose.position.x
        self.last_pos.y = cmd.pose.pose.position.y


        roll, pitch, yaw = tf.transformations.euler_from_quaternion([cmd.pose.pose.orientation.x, cmd.pose.pose.orientation.y, cmd.pose.pose.orientation.z, cmd.pose.pose.orientation.w])

        # dist=dist*1.1
        # yaw = yaw*1.05


        p.position.x = self.last_x + math.cos(yaw) * dist
        p.position.y = self.last_y + math.sin(yaw) * dist
        self.last_x=p.position.x
        self.last_y=p.position.y
        _tf = TransformStamped(
            header=Header(
                frame_id=cmd.header.frame_id,
                stamp=cmd.header.stamp
            ),
            child_frame_id=cmd.child_frame_id,
            transform=Transform(
               translation=cmd.pose.pose.position,
                rotation=cmd.pose.pose.orientation
            )
        )

        self.tf_pub.sendTransform(_tf)

# Start the node
if __name__ == '__main__':
    rospy.init_node("gazebo_odometry_node")
    node = OdometryNode()
    rospy.spin()
