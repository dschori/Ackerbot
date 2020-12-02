#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
import random

def callback(data, pub):
    # Here would be a place to do some sort of computation to decide if you want
    # the data republished on the new topic. I've inserted a dummy computation.:
    if random.randint(1,10) <= 5:
        rospy.loginfo("republish this cloud!")
        pub.publish(data)
    return

def listener():
    rospy.init_node('pointcloud_republisher')
    pub = rospy.Publisher("pcd_points", PointCloud2, queue_size=1)
    rospy.Subscriber("depth/points", PointCloud2, callback, callback_args=(pub))
    rospy.spin()

if __name__ == '__main__':
    listener()
