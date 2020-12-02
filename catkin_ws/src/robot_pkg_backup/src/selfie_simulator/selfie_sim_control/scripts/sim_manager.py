#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from pynput import keyboard

class msg_manager:
    # 1 -> manually     2-> semi semi_automat   3-> full automat
    def __init__(self, steering_mode = 1, force_stop = 0, msg_full_auto = AckermannDriveStamped(),
                msg_semi_auto = AckermannDriveStamped(),msg_manual = AckermannDriveStamped(), 
                default_left = 0.4, default_right = -0.4, speed = 0, steering_angle = 0):
        self.steering_mode = steering_mode
        self.force_stop = force_stop
        self.msg_full_auto = msg_full_auto
        self.msg_semi_auto = msg_semi_auto
        self.msg_manual = msg_manual
        self.default_speed = rospy.get_param("sim_speed", 0.4)
        rospy.logwarn('sim_speed = ' + str(self.default_speed))
        self.default_left = default_left
        self.default_right = default_right
        self.speed = speed
        self.steering_angle = steering_angle



    def pid_callback(self, data):
        # full automat
        if self.steering_mode == 3:
            self.msg_full_auto = data
        # semi automat
        elif self.steering_mode == 2:
            self.msg_semi_auto = data
            self.msg_semi_auto.drive.speed = self.speed

    def on_press(self, key):
        try:
            # space changes steering mode
            if key == keyboard.Key.space:
                if self.steering_mode == 1:
                    rospy.loginfo('SEMI_automat mode')
                    self.steering_mode = 2

                elif self.steering_mode == 2:
                    rospy.loginfo('FULL_automat mode')
                    self.steering_mode = 3

                elif self.steering_mode == 3:
                    rospy.loginfo('MANUAL mode')
                    self.steering_mode = 1
            if(key.char == 'r'):
                if(self.force_stop == 1):
                    self.speed = self.default_speed/2
            elif (key.char == 'e'):
                self.speed = -self.default_speed/2

   #         if (key.char == 'f'):
   #             if(self.force_stop == 0):
   #                 rospy.logwarn('stop mode')
   #                 self.force_stop = 1
   #             else:
   #                 rospy.logwarn('ride mode')
   #                 self.force_stop = 0

            elif(key.char == 'a'):
                if(self.force_stop == 1):
                    self.steering_angle = self.default_left
                    #self.speed = 0
                else:
                    #rospy.loginfo('left')
                    self.speed = self.default_speed/2
                    self.steering_angle = self.default_left
            elif(key.char =='d'):
                if(self.force_stop == 1):
                    self.steering_angle = self.default_right
                    #self.speed = 0
                else:
                    #rospy.loginfo('right')
                    self.speed = self.default_speed/2
                    self.steering_angle = self.default_right
            elif(key.char == 'w'):
                if(self.force_stop == 1):
                    self.steering_angle = self.default_left*2
                    #self.speed = 0
                else:
                    #rospy.loginfo('forward')
                    self.speed = self.default_speed
                    self.steering_angle = 0
            elif(key.char == 's'):
                if(self.force_stop == 1):
                    self.steering_angle = self.default_right*2
                    #self.speed = 0
                else:
                    #rospy.loginfo('backwards')
                    self.speed = - self.default_speed
                    self.steering_angle = 0

        except AttributeError:
            pass


    def on_release(self, key):
        try:
            if key.char == 'w' or key.char == 's' or key.char == 'd' or key.char == 'a' or  key.char == 'r' or key.char == 'e' or key.char == 'f' or key == keyboard.Key.space:
                self.speed = 0
                self.steering_angle = 0
            elif key == keyboard.Key.esc:
                rospy.signal_shutdown('closed by Esc')
                # Stop listener
                return False
        except AttributeError:
            pass

    def publish(self, pub):
        #rospy.loginfo("publish")
        if self.steering_mode == 1:
            self.msg_manual.drive.speed = self.speed
            self.msg_manual.drive.steering_angle = self.steering_angle
            self.msg_manual.drive.acceleration = 1
            self.msg_manual.drive.steering_angle_velocity = 10
            pub.publish(self.msg_manual)

        elif self.steering_mode == 2:
            #self.msg_semi_auto.drive.speed = 10
            pub.publish(self.msg_semi_auto)

        elif self.steering_mode == 3:
            pub.publish(self.msg_full_auto)

## main
if __name__ == '__main__':
    rospy.init_node('sim_manager', anonymous=True)
    manager = msg_manager()
    pub = rospy.Publisher('/sim_drive', AckermannDriveStamped, queue_size=1)
    rospy.Subscriber('/drive', AckermannDriveStamped, manager.pid_callback)

    listener = keyboard.Listener(
                on_press=manager.on_press,
                on_release=manager.on_release)
    listener.start()
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        manager.publish(pub)
        rate.sleep()

    rospy.signal_shutdown("manually closed")
