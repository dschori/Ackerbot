import math
import time
from pathlib import Path

# from sensor_msgs.msg import Image
# from tf.transformations import quaternion_from_euler
import matplotlib.pyplot as plt
import numpy as np
import rospy
import gym
from ackermann_msgs.msg import AckermannDriveStamped
from cv_bridge import CvBridge
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from gym import spaces
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
import tf
from sensor_msgs.msg import LaserScan, CompressedImage
from std_msgs.msg import Float64, String

default_sleep = 1


class ScoutingEnvInference(gym.Env):
    def __init__(self, env_config=None):
        super(ScoutingEnvInference).__init__()
        rospy.init_node('neuroracer_qlearn2', anonymous=True, log_level=rospy.INFO)
        self.initial_position = None

        self.min_distance = .55

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
        self.dyn1_x_min = -2.0
        self.dyn1_x_max = 1.2
        self.dyn1_last = 0.0
        self.dyn1_state = 0
        # self._check_publishers_connection()

        self.cumulated_steps = 0
        # self.observation_space = spaces.Box(low=0.0, high=8.0, shape=(12,))
        self.observation_space = spaces.Tuple((
            spaces.Box(low=0., high=4., shape=(18, )),
            spaces.Box(low=-10., high=10., shape=(2,)),
            spaces.Box(low=-1., high=1., shape=(2,)))
        )
        print(self.observation_space.sample())
        self.action_space = spaces.Box(low=-0.6, high=0.6, shape=(1, ), dtype=np.float32)
        self.rate = None
        self.speed = 1
        self.set_sleep_rate(100)
        self.number_of_sleeps = 10
        self.target_p = (-3.3, -1.7)
        self.last_d = 10.
        self.steerings = []
        self.offs_x, self.offs_y = 0., 0.
        self.last_p_x = 0.
        self.last_p_y = 0.
        rospy.logdebug("Finished NeuroRacerEnv INIT...")

    def set_sleep_rate(self, hz):
        self.rate = None
        if hz > 0:
            self.rate = rospy.Rate(hz)

    def _compute_reward(self, obs, action, done, finished):
        if finished:
            self.reward_publisher.publish(100.)
            return 100.
        if not done:
            reward1 = 0.
            pos_x, pos_y = self._get_pos_x_y()
            d = self._get_distance((pos_x, pos_y), (self.target_p[0], self.target_p[1]))
            p_x, p_y = abs(self.target_p[0] - pos_x), abs(self.target_p[1] - pos_y)
            if d < self.last_d:
                dt = self.last_d - d
                self.last_d = d
                reward1 = dt*2
            else:
                reward1 = -0.001

            ranges = self.get_laser_scan()
            if np.min(ranges) < 0.8:
                reward1 += -0.25 * (1 - np.min(ranges))

            self.reward_publisher.publish(reward1)
            return reward1
        else:
            self.reward_publisher.publish(-100.)
            return -100.

    def step(self, action):
        self.cumulated_steps += 1

        self.steerings.append(action[0]/2.)

        steering_angle = np.mean(self.steerings)
        self.speed = 0.3

        if len(self.steerings) > 5:
            _ = self.steerings.pop(0)

        steering_angle = action[0]/1.2
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
        for i in range(int(self.number_of_sleeps)):
            self.rate.sleep()
            self.steering(0.0, 0.0)

        while not rospy.is_shutdown():
            try:
                trans, rot = self.odom_listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
                print(trans)
                self.offs_x, self.offs_y = trans[0], trans[1]
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        self.last_p_x = self.offs_x
        self.last_p_y = self.offs_y
        return self._get_obs()

    def _check_all_systems_ready(self):
        """
        Checks that all the sensors, publishers and other simulation systems are
        operational.
        """
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
        """
        Checks that all the publishers are working
        :return:
        """
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

    def _get_obs(self):
        scan = self._process_scan()
        pos_x, pos_y = self._get_pos_x_y()
        t_x, t_y = self.target_p[0], self.target_p[1]
        p_x = t_x - pos_x
        p_y = t_y - pos_y

        # obs = np.append(scan, np.array([p_x, p_y]).reshape((1, 2)))

        scan = np.clip(scan.reshape((18, )), a_min=0.0, a_max=3.99)
        pos_to_target = np.clip(np.array([p_x, p_y]).reshape((2, )), a_min=-9.99, a_max=9.99).astype(np.float32)
        velocity = np.clip(np.array([self.last_p_x - p_x, self.last_p_y - p_y]).reshape((2, )), a_min=-.99, a_max=.99).astype(np.float32)
        self.last_p_x = p_x
        self.last_p_y = p_y
        self.target_pos_publisher.publish("t: {}, v: {}, steps: {}".format(pos_to_target, velocity, self.cumulated_steps))
        return (scan, pos_to_target, velocity)

    def _get_distance(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def _get_pos_x_y(self):
        #tfmessage = self.get_tfmessage()
        #px = tfmessage.transform.translation.x
        #py = tfmessage.transform.translation.y
        trans, rot = self.odom_listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
        px, py = trans[0], trans[1]
        #px += self.offs_x
        #py += self.offs_y
        return (px, py)

    def _process_scan(self):
        ranges = self.get_laser_scan().astype('float32')
        #distances = ranges.copy()
        #distances = np.clip(distances, 0.0, 4.0)
        #thetas = np.linspace(-3.14, 3.14, 897)
        #plt.figure()
        #plt.polar(thetas, distances)
        #plt.show()
        ranges = np.clip(ranges, 0.0, 4.0)
        ranges_chunks = np.array_split(ranges, 18)
        ranges_mean = np.array([np.min(chunk) for chunk in ranges_chunks])
        return ranges_mean.reshape(1, 18)

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
