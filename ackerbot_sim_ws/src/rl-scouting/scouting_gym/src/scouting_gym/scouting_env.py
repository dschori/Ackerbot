import time
import math
import numpy as np

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

from openai_ros import robot_gazebo_env
from sensor_msgs.msg import LaserScan, CompressedImage
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
# from std_msgs.msg import Float64
# from sensor_msgs.msg import Image
# from tf.transformations import quaternion_from_euler

from ackermann_msgs.msg import AckermannDriveStamped

from gym import spaces
from gym.envs.registration import register

# import cv2

default_sleep = 1


class ScoutingEnv(robot_gazebo_env.RobotGazeboEnv):
    def __init__(self):

        self.initial_position = None

        self.min_distance = .26

        self.last_int_difference = 0
        self.target_pos = (0.0, 2.0)

        self.bridge = CvBridge()

        # Doesnt have any accesibles
        self.controllers_list = []
        self.x_min, self.x_max, self.y_min, self.y_max = 0.0, 0.0, 0.0, 0.0
        self.max_area = 0.0
        # It doesnt use namespace
        self.robot_name_space = ""

        # We launch the init function of the Parent Class robot_gazebo_env.RobotGazeboEnv
        super(ScoutingEnv, self).__init__(controllers_list=self.controllers_list,
                                          robot_name_space=self.robot_name_space,
                                          reset_controls=False,
                                          start_init_physics_parameters=False)

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

        self.gazebo.unpauseSim()
        time.sleep(default_sleep)

        # self.controllers_object.reset_controllers()
        self._check_all_sensors_ready()

        self._init_camera()

        self.laser_subscription = rospy.Subscriber("/scan", LaserScan, self._laser_scan_callback)

        self.odom_subscription = rospy.Subscriber("ground_truth/state", Odometry, self._odom_callback)

        self.drive_control_publisher = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation",
                                                       AckermannDriveStamped,
                                                       queue_size=20)

        self._check_publishers_connection()

        self.gazebo.pauseSim()

        self.x_min, self.x_max, self.y_min, self.y_max = 0.0, 0.0, 0.0, 0.0
        self.max_area = 0.0
        rospy.logdebug("Finished NeuroRacerEnv INIT...")

    def _get_ini_position(self, init, not_p=None):
        if not_p is None:
            p = np.random.randint(0, 1)
        else:
            while True:
                p = np.random.randint(0, 5)
                if p != not_p:
                    break
        p_x, p_y, p_z = 0.0, 0.0, 0.05
        o_x, o_y, o_z, o_w = 0.0, 0.0, 0.75, 0.75
        if p == 0:
            p_x = np.random.uniform(-0.5, 0.5)
            p_y = np.random.uniform(0.0, 0.5)
        elif p == 1:
            p_x = np.random.uniform(-4.5, -4.2)
            p_y = np.random.uniform(-30, -25)
        elif p == 2:
            p_x = np.random.uniform(4.5, 5.0)
            p_y = np.random.uniform(-25, -24)
        elif p == 3:
            p_x = np.random.uniform(4.5, 5.0)
            p_y = np.random.uniform(-20, -21)
        elif p == 4:
            p_x = np.random.uniform(2.0, 2.5)
            p_y = np.random.uniform(8.0, 9.0)

        if init:
            return {'p_x': p_x, 'p_y': p_y, 'p_z': p_z, 'o_x': o_x,
                    'o_y': o_y, 'o_z': o_z, 'o_w': o_w}, p
        else:
            return (p_x, p_y), p

    def reset_position(self):
        if not self.initial_position:
            return
        state_msg = ModelState()
        state_msg.model_name = 'racecar'
        state_msg.pose.position.x = self.initial_position['p_x']
        state_msg.pose.position.y = self.initial_position['p_y']
        state_msg.pose.position.z = self.initial_position['p_z']
        state_msg.pose.orientation.x = self.initial_position['o_x']
        state_msg.pose.orientation.y = self.initial_position['o_y']
        state_msg.pose.orientation.z = self.initial_position['o_z']
        state_msg.pose.orientation.w = self.initial_position['o_w']

        self.set_model_state(state_msg)

    def reset(self):
        super(ScoutingEnv, self).reset()
        self.steps_count = 0
        # self.initial_position = {'p_x': 2, 'p_y': np.random.uniform(8, 9), 'p_z': 0.05, 'o_x': 0,
        #                         'o_y': 0.0, 'o_z': np.random.uniform(-1.5, -2.0), 'o_w': -0.8}
        self.initial_position, p = self._get_ini_position(init=True)
        self.target_pos, p = self._get_ini_position(init=False)
        self.x_min, self.x_max, self.y_min, self.y_max = self.initial_position['p_x'] - 0.01, self.initial_position[
            'p_x'] + 0.01, self.initial_position['p_y'] - 0.01, self.initial_position['p_y'] + 0.01
        self.max_area = 0.0
        self.last_int_difference = self._get_distance(self.target_pos,
                                                      (self.initial_position['p_x'], self.initial_position['p_y']))
        self.gazebo.unpauseSim()
        self.reset_position()

        time.sleep(default_sleep)
        self.gazebo.pauseSim()

        return self._get_obs()

    def _check_all_systems_ready(self):
        """
        Checks that all the sensors, publishers and other simulation systems are
        operational.
        """
        self._check_all_sensors_ready()
        return True

    # virtual methods
    # ----------------------------

    def _check_all_sensors_ready(self):
        rospy.logdebug("START ALL SENSORS READY")
        self._check_laser_scan_ready()
        self._check_camera_ready()
        self._check_odom_ready()
        rospy.logdebug("ALL SENSORS READY")

    def _check_odom_ready(self):
        self.odom = None
        rospy.logdebug("Waiting for ground_truth/state to be READY...")
        while self.odom is None and not rospy.is_shutdown():
            try:
                self.odom = rospy.wait_for_message('ground_truth/state',
                                                   Odometry,
                                                   timeout=1.0)
            except:
                rospy.logerr("Odom not ready yet, retrying for getting odom_msg")

    def _check_camera_ready(self):
        self.camera_msg = None
        rospy.logdebug("Waiting for /camera/zed/rgb/image_rect_color/compressed to be READY...")
        while self.camera_msg is None and not rospy.is_shutdown():
            try:
                self.camera_msg = rospy.wait_for_message('/camera/zed/rgb/image_rect_color/compressed',
                                                         CompressedImage,
                                                         timeout=1.0)
            except:
                rospy.logerr("Camera not ready yet, retrying for getting camera_msg")

    def _init_camera(self):
        img = self.get_camera_image()

        # self.color_scale = "bgr8" # config["color_scale"]
        self.input_shape = (300,)
        # obs_low = np.append(np.zeros(300), np.array((-100., -100)))
        # obs_high = np.append(np.ones(300) * 10., np.array((100., 100)))
        obs_low = 0.0
        obs_high = 10.0
        self.observation_space = spaces.Box(low=obs_low, high=obs_high, shape=self.input_shape)
        # self.observation_space = spaces.Discrete(low=obs_low, high=obs_high, shape=self.input_shape)

        img_dims = img.shape[0] * img.shape[1] * img.shape[2]
        byte_size = 4
        overhaead = 2  # reserving memory for ros header
        buff_size = img_dims * byte_size * overhaead
        self.camera_msg = rospy.Subscriber("/camera/zed/rgb/image_rect_color/compressed",
                                           CompressedImage, self._camera_callback, queue_size=1,
                                           buff_size=buff_size)
        rospy.logdebug("== Camera READY ==")

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

    #     def _get_additional_laser_scan(self):
    #         laser_scans = []
    #         self.gazebo.unpauseSim()
    #         while len(laser_scans) < 2  and not rospy.is_shutdown():
    #             try:
    #                 data = rospy.wait_for_message("/scan", LaserScan, timeout=1.0)
    #                 laser_scans.append(data.ranges)
    #             except Exception as e:
    #                 rospy.logerr("getting laser data...")
    #                 print(e)
    #         self.gazebo.pauseSim()

    #         return laser_scans

    def _laser_scan_callback(self, data):
        self.laser_scan = data

    def _camera_callback(self, msg):
        self.camera_msg = msg

    def _odom_callback(self, msg):
        self.odom = msg

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

    def _set_init_pose(self):
        """Sets the Robot in its init pose
        """
        raise NotImplementedError()

    def _init_env_variables(self):
        """Inits variables needed to be initialised each time we reset at the start
        of an episode.
        """
        raise NotImplementedError()

    def _compute_reward(self, observations, done):
        """Calculates the reward to give based on the observations given.
        """
        raise NotImplementedError()

    def _set_action(self, action):
        """Applies the given action to the simulation.
        """
        raise NotImplementedError()

    def _get_obs(self):
        scan = self._process_scan()
        # scan = self.get_laser_scan().astype('float32')
        # scan = np.clip(scan, None, 10.0) / 10.0
        # scan = scan.reshape((1000, ))
        (x, y) = self._get_pos_x_y()
        if self.initial_position:
            x -= self.initial_position['p_x']
            y -= self.initial_position['p_y']
        # (ox, oy, oz, ow) = self._get_orientation()
        return scan
        # return np.append(scan, np.array((x, y)))

    def get_odom(self):
        return self.odom

    def _get_orientation(self):
        odom = self.get_odom()
        ox = odom.pose.pose.orientation.x
        oy = odom.pose.pose.orientation.y
        oz = odom.pose.pose.orientation.z
        ow = odom.pose.pose.orientation.w

        # ox = np.clip((ox + 1.0) / 2.0, 0.0, 1.0)
        # oy = np.clip((oy + 1.0) / 2.0, 0.0, 1.0)
        # oz = np.clip((oz + 1.0) / 2.0, 0.0, 1.0)
        # ow = np.clip((ow + 1.0) / 2.0, 0.0, 1.0)
        return (ox, oy, oz, ow)

    def _get_distance(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def _get_pos_x_y(self):
        odom = self.get_odom()
        px = odom.pose.pose.position.x
        py = odom.pose.pose.position.y
        return (px, py)

    def _process_scan(self):
        ranges = self.get_laser_scan().astype('float32')
        ranges = np.clip(ranges, 0.0, 10.0)
        ranges_chunks = np.array_split(ranges, 300)
        ranges_mean = np.array([np.min(chunk) for chunk in ranges_chunks])
        return ranges_mean.reshape(300, )

    def _is_done(self, observations):
        self._episode_done = self._is_collided()
        return self._episode_done

    def _is_finished(self, obs):
        if self.steps_count > 1200:
            return True
        else:
            return False

    def _create_steering_command(self, steering_angle, speed):
        # steering_angle = np.clip(steering_angle,self.steerin_angle_min, self.steerin_angle_max)

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

    # def get_odom(self):
    #     return self.odom

    # def get_imu(self):
    #     return self.imu

    def get_laser_scan(self):
        return np.array(self.laser_scan.ranges, dtype=np.float32)

    def get_camera_image(self):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(self.camera_msg).astype('float32')
        except Exception as e:
            rospy.logerr("CvBridgeError: Error converting image")
            rospy.logerr(e)
        return cv_image

    def _is_collided(self):
        r = self.get_laser_scan()
        crashed = np.any(r <= self.min_distance)
        if crashed:
            #             rospy.logdebug('the auto crashed! :(')
            #             rospy.logdebug('distance: {}'.format(r.min()))
            #             data = np.array(self._get_additional_laser_scan(), dtype=np.float32)
            #             data = np.concatenate((np.expand_dims(r, axis=0), data), axis=0)
            #             data_mean = np.mean(data, axis=0)
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
