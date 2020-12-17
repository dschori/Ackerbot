#!/usr/bin/env python
import rospy, math
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from adafruit_servokit import ServoKit
from simple_pid import PID
import time

rospy.init_node('drive_controller2')

class Controller:
    def __init__(self):
        self.kit = ServoKit(channels=16)

        self.speed_cmd = 0.0
        self.speed_sensor = 0.0

        self.steering_cmd = 0.0
        self.steering_sensor = 0.0

        self.pid_speed = PID(0.1, 4, 0.0, setpoint=0)
        self.pid_speed.sample_time = 0.01
        self.pid_speed.output_limits = (-1.0, 1.0)

        self.pid_steering = PID(0.2, 6, 0.0, setpoint=0)
        self.pid_steering.sample_time = 0.01
        self.pid_steering.output_limits = (-1.0, 1.0)

        rospy.loginfo('Drive Controller Started')

    def cmd_callback(self, msg):
        self.steering_cmd = msg.drive.steering_angle
        self.speed_cmd = msg.drive.speed

        self.pid_speed.setpoint = self.speed_cmd
        self.pid_steering.setpoint = self.steering_cmd

    def sensor_callback(self, msg):
        self.speed_sensor = msg.twist.twist.linear.x
        self.steering_sensor = msg.twist.twist.angular.z

        if abs(self.speed_sensor) < 0.04:
            self.speed_sensor = 0.0

        if abs(self.steering_sensor) < 0.04 and abs(self.speed_sensor) < 0.04:
            self.steering_sensor = 0.0

    def translate(self, value, leftMin, leftMax, rightMin, rightMax):
        # Figure out how 'wide' each range is
        leftSpan = leftMax - leftMin
        rightSpan = rightMax - rightMin

        # Convert the left range into a 0-1 range (float)
        valueScaled = float(value - leftMin) / float(leftSpan)

        # Convert the 0-1 range into a value in the right range.
        return rightMin + (valueScaled * rightSpan)

    def controll(self):
        rospy.Subscriber("/camera_t265/odom/sample", Odometry, self.sensor_callback)
        rospy.Subscriber("/ackermann_cmd", AckermannDriveStamped, self.cmd_callback)
        current_time = time.time()
        while not rospy.is_shutdown():
            output_speed = self.pid_speed(self.speed_sensor)
            self.kit.continuous_servo[15].throttle = -output_speed

            output_steering = self.pid_steering(self.steering_sensor)
            if self.speed_sensor < 0.0:
                output_steering = output_steering * -1
            angle = self.translate(output_steering, 1.0, -1.0, 75, 175)
            self.kit.servo[0].angle = angle

            if time.time() - current_time > 0.5:
                print('speed: {}, steering: {}'.format(output_speed, output_steering))
                current_time = time.time()

            rospy.sleep(0.02)

if __name__ == '__main__':
    c = Controller()
    c.controll()
