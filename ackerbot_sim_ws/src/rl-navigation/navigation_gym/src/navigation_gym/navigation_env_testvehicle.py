import math
import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import rospy
import gym
from ackermann_msgs.msg import AckermannDriveStamped
from cv_bridge import CvBridge
from gym import spaces
from tf2_msgs.msg import TFMessage
import tf
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64, String
import cv2
from tf.transformations import euler_from_quaternion, quaternion_from_euler

default_sleep = 1


class ScoutingEnvInference(gym.Env):
    def __init__(self, env_config=None):
        self.obs_save_ind = 0
        self.obs_images = np.zeros((84, 84, 4))
        self.max_lidar_range = 5.
        self.img_size = 84
        super(ScoutingEnvInference).__init__()
        rospy.init_node('ackerbot', anonymous=True, log_level=rospy.INFO)
        self.initial_position = None

        self.min_distance = .5

        self.last_int_difference = 0
        self.target_pos = (0.0, 2.0)

        self.bridge = CvBridge()

        time.sleep(default_sleep)

        self._check_all_sensors_ready()

        self.laser_subscription = rospy.Subscriber("/scan", LaserScan, self._laser_scan_callback)

        self.odom_subscription = rospy.Subscriber("/tf", TFMessage, self._tfmessage_callback)

        self.odom_listener = tf.TransformListener()

        self.drive_control_publisher = rospy.Publisher("/ackermann_cmd",
                                                       AckermannDriveStamped,
                                                       queue_size=20)

        self.reward_publisher = rospy.Publisher("/env_reward",
                                                Float64,
                                                queue_size=20)

        self.target_pos_publisher = rospy.Publisher("/pos_to_target",
                                                    String,
                                                    queue_size=20)

        self.cumulated_steps = 0
        self.observation_space = spaces.Box(low=0.0, high=1.0, shape=(self.img_size, self.img_size, 4))
        self.action_space = spaces.Discrete(3)
        self.rate = None
        self.speed = 1
        self.set_sleep_rate(100)
        self.number_of_sleeps = 25
        self.target_p = (-3.3, -1.7)
        self.last_d = 10.
        self.steerings = []
        self.offs_x, self.offs_y = 0., 0.
        self.last_p_x = 0.
        self.last_p_y = 0.
        self.img_prefix = ""

    def set_sleep_rate(self, hz):
        self.rate = None
        if hz > 0:
            self.rate = rospy.Rate(hz)

    def _compute_reward(self, obs, action, done, finished):
        reward1, reward2 = .0, .0

        # If goal reached, return here:
        if finished:
            reward = 100.
            self.reward_publisher.publish(reward)
            return reward

        # If crashed, return here:
        if done:
            reward = -100.
            self.reward_publisher.publish(reward)
            return reward

        # Else ,cumulate distance to target and distance to obstacle reward
        pos_x, pos_y = self._get_pos_x_y()

        d = self._get_distance((pos_x, pos_y), (self.target_p[0], self.target_p[1]))

        reward1 = 2.*(self.last_d - d)
        self.last_d = d

        ranges = self.get_laser_scan()
        if np.min(ranges) < 1.0:
            reward2 = -0.5 * (1. - np.min(ranges))

        self.reward_publisher.publish(reward1 + reward2)
        return reward1 + reward2

    def step(self, action):
        self.cumulated_steps += 1

        self.speed = 0.3

        steering_angle = 0.
        if action == 0:
            steering_angle = -0.9
        elif action == 1:
            steering_angle = 0.
        elif action == 2:
            steering_angle = 0.9

        self.last_action = action
        self.steering(steering_angle, self.speed)
        if self.rate:
            for i in range(int(self.number_of_sleeps)):
                self.rate.sleep()
                self.steering(steering_angle, self.speed)

        obs = self._get_obs()
        reward = self._compute_reward(obs, action, self._is_done(obs), self._is_finished(obs))
        done = False
        if self._is_done(obs) or self._is_finished(obs):
            done = True
        return obs, reward, done, {}

    def reset(self):
        self.last_d = 10.
        self.cumulated_steps = 0
        self.obs_images = np.zeros((84, 84, 4))
        for i in range(int(1)):
            self.rate.sleep()
            self.steering(0.0, 0.0)

        while not rospy.is_shutdown():
            try:
                trans, rot = self.odom_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                self.offs_x, self.offs_y = trans[0], trans[1]
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        self.last_p_x = self.offs_x
        self.last_p_y = self.offs_y
        return self._get_obs()

    def _check_all_systems_ready(self):
        self._check_all_sensors_ready()
        return True

    def _check_all_sensors_ready(self):
        rospy.logdebug("START ALL SENSORS READY")
        self._check_laser_scan_ready()
        self._check_odom_ready()
        rospy.logdebug("ALL SENSORS READY")

    def _check_odom_ready(self):
        self.tfmessage = None
        rospy.logdebug("Waiting for tf to be READY...")
        while self.tfmessage is None and not rospy.is_shutdown():
            try:
                self.tfmessage = rospy.wait_for_message('/tf',
                                                        TFMessage,
                                                        timeout=1.0)
            except:
                rospy.logerr("TF not ready yet, retrying for getting tf msg")

    def _check_laser_scan_ready(self):
        self.laser_scan = None
        rospy.logdebug("Waiting for /scan to be READY...")
        while self.laser_scan is None and not rospy.is_shutdown():
            try:
                self.laser_scan = rospy.wait_for_message("/scan", LaserScan, timeout=1.0)
                rospy.logdebug("Current /scan READY=>")

            except:
                rospy.logerr("Current /scan not ready yet, retrying for getting laser_scan")
        return self.laser_scan

    def _laser_scan_callback(self, data):
        self.laser_scan = data

    def _camera_callback(self, msg):
        self.camera_msg = msg

    def _tfmessage_callback(self, msg):
        self.tfmessage = msg

    def _check_publishers_connection(self):
        rate = rospy.Rate(10)  # 10hz
        while self.drive_control_publisher.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.logdebug("No susbribers to drive_control_publisher yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("drive_control_publisher Publisher Connected")

        rospy.logdebug("All Publishers READY")

    def _encode_state(self, laser_scan, rel_target_position):
        image = self.laserscan_to_image(laser_scan)
        robot = np.zeros((self.img_size, self.img_size, 3), dtype=np.uint8)
        robot = cv2.circle(robot, (self.img_size // 2, self.img_size // 2), radius=2, color=(255, 0, 0), thickness=2)
        image += robot
        pix_x, pix_y = rel_target_position
        image = cv2.circle(image, (pix_x, pix_y), radius=2, color=(0, 255, 0), thickness=3)
        image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        image = image.astype(np.float32)
        image /= 255.

        image /= image.max()
        image = np.clip(image, 0.0, 1.0)

        plt.imsave('{}/img_logs/{}_obs_{}.png'.format(Path.home(), self.img_prefix, self.obs_save_ind + 1000000), image)
        return image

    def _get_obs(self):

        def pol2cart(rho, phi):
            x = rho * np.cos(phi)
            y = rho * np.sin(phi)
            return x, y

        def target_to_pixels(target_x, target_y, lidar_range):
            target_x = max(min(target_x, lidar_range / 2.), -lidar_range / 2.)
            target_y = max(min(target_y, lidar_range / 2.), -lidar_range / 2.)
            pix_x = int(((self.img_size // 2 - 4) / (lidar_range / 2.)) * target_x)
            pix_y = int(((self.img_size // 2 - 4) / (lidar_range / 2.)) * target_y)
            return pix_x + self.img_size // 2, pix_y + self.img_size // 2

        def get_target_coords(dist, phi, lidar_range):
            x, y = pol2cart(dist / 2., phi)
            x, y = target_to_pixels(x, y, lidar_range)
            return x, y

        pos_x, pos_y = self._get_pos_x_y()
        target_x, target_y = self.target_p[0], self.target_p[1]
        distance_to_target = self._get_distance((pos_x, pos_y), (target_x, target_y))
        yaw_of_robot = self._get_angle_z()
        dx = pos_x - target_x
        dy = pos_y - target_y
        angle_to_target = np.arctan2(dx, dy)

        pix_x, pix_y = get_target_coords(dist=distance_to_target, phi=yaw_of_robot + angle_to_target,
                                         lidar_range=self.max_lidar_range)

        image = self._encode_state(self.laser_scan, (pix_x, pix_y))

        self.obs_save_ind += 1
        self.obs_images = np.append(self.obs_images, image.reshape((84, 84, 1)), axis=2)
        self.obs_images = np.delete(self.obs_images, 0, axis=2)

        return np.flip(self.obs_images, axis=2)

    def _get_angle_z(self):
        while True:
            try:
                trans, rot = self.odom_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                self.offs_x, self.offs_y = trans[0], trans[1]
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        orientation_list = [rot[0],
                            rot[1],
                            rot[2],
                            rot[3]]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        return yaw

    def _get_distance(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def _get_pos_x_y(self):
        while True:
            try:
                trans, rot = self.odom_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                self.offs_x, self.offs_y = trans[0], trans[1]
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        px, py = trans[0], trans[1]
        return (px, py)

    def _is_done(self, observations):
        self._episode_done = self._is_collided()
        if self._episode_done or self.cumulated_steps > 600:
            return True
        else:
            return False

    def _is_finished(self, obs):
        pos_x, pos_y = self._get_pos_x_y()
        d = self._get_distance((pos_x, pos_y), (self.target_p[0], self.target_p[1]))
        p_x, p_y = abs(self.target_p[0] - pos_x), abs(self.target_p[1] - pos_y)
        if d < 0.6:
            return True
        else:
            return False

    def _create_steering_command(self, steering_angle, speed):
        a_d_s = AckermannDriveStamped()
        a_d_s.drive.steering_angle = steering_angle
        a_d_s.drive.steering_angle_velocity = 0.0
        a_d_s.drive.speed = speed  # from 0 to 1
        a_d_s.drive.acceleration = 0.0
        a_d_s.drive.jerk = 0.0

        return a_d_s

    def steering(self, steering_angle, speed):
        command = self._create_steering_command(steering_angle, speed)
        self.drive_control_publisher.publish(command)

    def get_laser_scan(self):
        return np.array(self.laser_scan.ranges, dtype=np.float32)

    def _is_collided(self):
        r = self.get_laser_scan()
        crashed = np.any(r <= self.min_distance)
        if crashed:
            min_range_idx = r.argmin()
            min_idx = min_range_idx - 5
            if min_idx < 0:
                min_idx = 0
            max_idx = min_idx + 10
            if max_idx >= r.shape[0]:
                max_idx = r.shape[0] - 1
                min_idx = max_idx - 10
            mean_distance = r[min_idx:max_idx].mean()

            crashed = np.any(mean_distance <= self.min_distance)

        return crashed

    def laserscan_to_image(self, scan):
        # based on stackoverflow code
        # Discretization Size
        disc_size = .2
        # Discretization Factor
        disc_factor = 1 / disc_size
        # Max Lidar Range
        max_lidar_range = self.max_lidar_range
        # Create Image Size Using Range and Discretization Factor
        image_size = int(max_lidar_range * 2 * disc_factor)

        # Store maxAngle of lidar
        maxAngle = scan.angle_max
        # Store minAngle of lidar
        minAngle = scan.angle_min
        # Store angleInc of lidar
        angleInc = scan.angle_increment
        # Store maxLength in lidar distances
        maxLength = scan.range_max
        # Store array of ranges
        ranges = scan.ranges
        ranges = np.asarray(ranges)
        ranges[ranges == np.inf] = .35

        ranges = np.clip(ranges, 0.0, max_lidar_range)

        # Calculate the number of points in array of ranges
        num_pts = len(ranges)
        # Create Array for extracting X,Y points of each data point
        xy_scan = np.zeros((num_pts, 2))
        # Create 3 Channel Blank Image
        blank_image = np.zeros((image_size, image_size, 3), dtype=np.uint8)
        blank_image[:, :, 2] = 255
        # Loop through all points converting distance and angle to X,Y point
        for i in range(num_pts):
            # Check that distance is not longer than it should be
            if (ranges[i] > max_lidar_range) or (math.isnan(ranges[i])):
                pass
            else:
                # Calculate angle of point and calculate X,Y position
                angle = minAngle + float(i) * angleInc
                xy_scan[i][0] = float(ranges[i] * math.cos(angle))
                xy_scan[i][1] = float(ranges[i] * math.sin(angle))
                b_i = 0.0
                while True:
                    pt_x = float(b_i * math.cos(angle))
                    pt_y = float(b_i * math.sin(angle))
                    pix_x = int(math.floor((pt_x + max_lidar_range) * disc_factor))
                    pix_y = int(math.floor((max_lidar_range - pt_y) * disc_factor))
                    if b_i >= ranges[i]:
                        break
                    try:
                        blank_image[pix_y, pix_x] = [0, 0, 0]
                    except IndexError:
                        break
                    b_i += 0.2

        blank_image = cv2.rotate(blank_image, cv2.cv2.ROTATE_90_COUNTERCLOCKWISE)
        blank_image = cv2.resize(blank_image, (84, 84), interpolation=cv2.INTER_AREA)
        # Convert CV2 Image to ROS Message
        img = self.bridge.cv2_to_imgmsg(blank_image, encoding="bgr8")
        # Publish image
        return blank_image
