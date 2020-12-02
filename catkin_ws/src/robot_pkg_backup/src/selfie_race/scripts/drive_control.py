import rospy
import roslib
from geometry_msgs.msg import Twist

from adafruit_servokit import ServoKit
kit = ServoKit(channels=16)

rospy.init_node('control_subscriver')

r = rospy.Rate(1.0)


class Control:
    def __init__(self):
        self.pos_x, self.pos_y = 0., 0.
        self.ori_x, self.ori_y, self.ori_z, self.ori_w = 0., 0., 0., 0.
        self.vel_lin_x, self.vel_lin_y, self.vel_lin_z = 0., 0., 0.
        self.vel_ang_x, self.vel_ang_y, self.vel_ang_z = 0., 0., 0.

    def keyboard_twist(self, msg):

        self.vel_lin_x = msg.twist.twist.linear.x
        self.vel_lin_y = msg.twist.twist.linear.y
        self.vel_lin_z = msg.twist.twist.linear.z

        self.vel_ang_x = msg.twist.twist.angular.x
        self.vel_ang_y = msg.twist.twist.angular.y
        self.vel_ang_z = msg.twist.twist.angular.z

        kit.continuous_servo[15].throttle = self.vel_lin_x

    def listener(self):
        rospy.Subscriber("/cmd_vel", Twist, self.keyboard_twist)
        rospy.sleep(0.01)


if __name__ == '__main__':
    c = Control()
    c.listener()