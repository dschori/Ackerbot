#!/usr/bin/env python
import rospy, math
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from adafruit_servokit import ServoKit

rospy.init_node('drive_controller')

r = rospy.Rate(1.0)

class Controller:
    def __init__(self):
        self.kit = ServoKit(channels=16)
        self.steering_angle = 0.0
        self.speed = 0.0
        rospy.loginfo('Drive Controller Started')

    def ackermann_cmd_sub(self, msg):
        self.steering_angle = msg.drive.steering_angle
        self.speed = msg.drive.speed
        self.steering_angle = min(1, self.steering_angle*1.25) 
        self.steering_angle = max(-1, self.steering_angle*1.25)
        if self.speed > 0.02:
            self.speed = max(0.35, self.speed)
        if self.speed < -0.02:
            self.speed = min(-0.35, self.speed)
        self.speed = min(0.8, self.speed)
        self.speed = max(-0.8, self.speed)

    def translate(self, value, leftMin, leftMax, rightMin, rightMax):
        # Figure out how 'wide' each range is
        leftSpan = leftMax - leftMin
        rightSpan = rightMax - rightMin

        # Convert the left range into a 0-1 range (float)
        valueScaled = float(value - leftMin) / float(leftSpan)

        # Convert the 0-1 range into a value in the right range.
        return rightMin + (valueScaled * rightSpan)

    def listener(self):
        current_time = rospy.Time.now()
        last_time = rospy.Time.now()
        rospy.Subscriber("/ackermann_cmd", AckermannDriveStamped, self.ackermann_cmd_sub, queue_size=1)
        last_angle = 110
        last_speed = 0.0
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            angle = self.translate(self.steering_angle, 1.0, -1.0, 75, 175)
            speed = -self.speed
            for angle_value, speed_value in zip(np.linspace(last_angle, angle, 10), 
                                                np.linspace(last_speed, speed, 10)):
                self.kit.servo[0].angle = angle_value
                self.kit.continuous_servo[15].throttle = speed_value
                rospy.sleep(0.03)

            last_angle = angle
            last_speed = speed
            
            


if __name__ == '__main__':
    c = Controller()
    c.listener()
