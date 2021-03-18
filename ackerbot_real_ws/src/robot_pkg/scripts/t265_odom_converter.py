#!/usr/bin/env python
import rospy 
import roslib
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from geometry_msgs.msg import Twist
import time
import math
import tf
from math import sin, cos, pi

odom_pub = rospy.Publisher("odom_t265", Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()

rospy.init_node('odometry_publisher')

r = rospy.Rate(1.0)

x, y, = 0., 0.

class Converter:
    def __init__(self):
        self.pos_x, self.pos_y = 0., 0.
        self.ori_x, self.ori_y, self.ori_z, self.ori_w = 0., 0., 0., 0.
        self.vel_lin_x, self.vel_lin_y, self.vel_lin_z = 0., 0., 0.
        self.vel_ang_x, self.vel_ang_y, self.vel_ang_z = 0., 0., 0.

    def odometryCb(self, msg):
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
        v_x = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        #q0 = msg.pose.pose.orientation.w
        #q1 = msg.pose.pose.orientation.x
        #q2 = msg.pose.pose.orientation.y
        #q3 = msg.pose.pose.orientation.z
        self.ori_w = msg.pose.pose.orientation.w
        self.ori_x = msg.pose.pose.orientation.x
        self.ori_y = msg.pose.pose.orientation.y
        self.ori_z = msg.pose.pose.orientation.z

        self.vel_lin_x = msg.twist.twist.linear.x
        self.vel_lin_y = msg.twist.twist.linear.y
        self.vel_lin_z = msg.twist.twist.linear.z

        self.vel_ang_x = msg.twist.twist.angular.x
        self.vel_ang_y = msg.twist.twist.angular.y
        self.vel_ang_z = msg.twist.twist.angular.z

        #yaw = math.degrees(math.atan2(2.0*(q0*q3 + q1*q2),(1.0-2.0*(q2*q2 + q3*q3))))
        t= 1.0*msg.header.stamp.secs + 1.0*(msg.header.stamp.nsecs)/1000000000
        #row = [ v_x, w, yaw, t]


    def listener(self):
        current_time = rospy.Time.now()
        last_time = rospy.Time.now()
        rospy.Subscriber("camera_t/odom/sample", Odometry, self.odometryCb)
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()

            # since all odometry is 6DOF we'll need a quaternion created from yaw
            odom_euler = tf.transformations.euler_from_quaternion([self.ori_x, self.ori_y, self.ori_z, self.ori_w])
            
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, odom_euler[2])
            
            #odom_quat = [self.ori_x, self.ori_y, self.ori_z, self.ori_w]
            #odom_quat = [0, 0, 0, 0]

            # first, we'll publish the transform over tf
            odom_broadcaster.sendTransform(
                (self.pos_x, self.pos_y, 0.),
                odom_quat,
                current_time,
                "base_link",
                "odom_t265"
            )

            # next, we'll publish the odometry message over ROS
            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = "odom_t265"

            # set the position
            odom.pose.pose = Pose(Point(self.pos_x, self.pos_y, 0.), Quaternion(*odom_quat))

            # set the velocity
            odom.child_frame_id = "base_link"
            odom.twist.twist = Twist(Vector3(self.vel_lin_x, self.vel_lin_y, self.vel_lin_z), Vector3(self.vel_ang_x, self.vel_ang_y, self.vel_ang_z))

            # publish the message
            odom_pub.publish(odom)

            last_time = current_time
            rospy.sleep(0.01)

if __name__ == '__main__':
    c = Converter()
    c.listener()