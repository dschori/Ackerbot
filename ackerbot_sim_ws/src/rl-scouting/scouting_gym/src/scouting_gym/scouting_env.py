import time
import math
import numpy as np

from scipy.stats import norm

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

from openai_ros import robot_gazebo_env
from sensor_msgs.msg import LaserScan, CompressedImage
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float64, String
# from sensor_msgs.msg import Image
# from tf.transformations import quaternion_from_euler
import matplotlib.pyplot as plt
from pathlib import Path

from ackermann_msgs.msg import AckermannDriveStamped

from gym import spaces
from gym.envs.registration import register

# import cv2

default_sleep = 1


class ScoutingEnv(robot_gazebo_env.RobotGazeboEnv):
    def __init__(self):

        self.initial_position = None

        self.min_distance = .55

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

        self._check_all_sensors_ready()

        self._init_camera()

        self.laser_subscription = rospy.Subscriber("/scan", LaserScan, self._laser_scan_callback)

        self.odom_subscription = rospy.Subscriber("ground_truth/state", Odometry, self._odom_callback)

        self.drive_control_publisher = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation",
                                                       AckermannDriveStamped,
                                                       queue_size=20)

        self.reward_publisher = rospy.Publisher("/env_reward",
                                                Float64,
                                                queue_size=20)

        self.target_pos_publisher = rospy.Publisher("/pos_to_target",
                                                    String,
                                                    queue_size=20)

        self.dyn1_publisher = rospy.Publisher("/gazebo/set_model_state",
                                              ModelState,
                                              queue_size=20)

        self.scan_publisher = rospy.Publisher("/obs_image",
                                              CompressedImage,
                                              queue_size=20)
        self.dyn1_x_min = -2.0
        self.dyn1_x_max = 1.2
        self.dyn1_last = 0.0
        self.dyn1_state = 0

        self._check_publishers_connection()

        self.gazebo.pauseSim()

        self.x_min, self.x_max, self.y_min, self.y_max = 0.0, 0.0, 0.0, 0.0
        self.max_area = 0.0
        self.obs_save_ind = 0
        self.target_p = (5., 7.)
        self.cumulated_steps = 0
        self.last_p_x = 0.
        self.last_p_y = 0.
        rospy.logdebug("Finished NeuroRacerEnv INIT...")

    def _update_dyn1(self):
        if self.dyn1_state == 0:
            self.dyn1_last += 0.01
            if self.dyn1_last > self.dyn1_x_max:
                self.dyn1_state = 1
        else:
            self.dyn1_last -= 0.01
            if self.dyn1_last < self.dyn1_x_min:
                self.dyn1_state = 0

        ms = ModelState()
        ms.model_name = 'unit_cylinder'
        ms.reference_frame = 'world'
        ms.pose.position.x = self.dyn1_last
        self.dyn1_publisher.publish(ms)

    def _get_ini_and_target_position(self):

        env = np.random.randint(0, 3)
        p_x, p_y, p_z = 0.0, 0.0, 0.05
        o_x, o_y, o_z, o_w = 0.0, 0.0, 0.75, 0.75
        if env == 0:
            choice = np.random.randint(0, 2)
            if choice == 0:
                p_x = np.random.uniform(-0.5, -1.0)
                p_y = np.random.uniform(-4.3, -4.5)
                t_x = np.random.uniform(-2.5, -3.0)
                t_y = np.random.uniform(2.5, 3.0)
            else:
                p_x = np.random.uniform(-2.5, -3.0)
                p_y = np.random.uniform(2.5, 3.0)
                t_x = np.random.uniform(-0.5, -1.0)
                t_y = np.random.uniform(-4.3, -4.5)
            ini_pos = {'p_x': p_x, 'p_y': p_y, 'p_z': p_z, 'o_x': o_x,
                       'o_y': o_y, 'o_z': 1.5, 'o_w': 1.5}
            target_pos = (t_x, t_y)
            return ini_pos, target_pos

        elif env == 1:
            choice = np.random.randint(0, 2)
            if choice == 0:
                p_x = np.random.uniform(18.0, 18.5)
                p_y = np.random.uniform(-4.5, -5.)
                # t_x = np.random.uniform(10., 10.5)
                # t_y = np.random.uniform(5.5, 6.0)
                t_x = np.random.uniform(10., 10.5)
                t_y = np.random.uniform(1.5, 2.0)
            else:
                # p_x = np.random.uniform(10., 10.5)
                # p_y = np.random.uniform(5.5, 6.0)
                p_x = np.random.uniform(10., 10.5)
                p_y = np.random.uniform(1.5, 2.5)
                t_x = np.random.uniform(18.0, 18.5)
                t_y = np.random.uniform(-4.5, -5.)

            ini_pos = {'p_x': p_x, 'p_y': p_y, 'p_z': p_z, 'o_x': o_x,
                       'o_y': o_y, 'o_z': 3.4, 'o_w': 1.5}
            target_pos = (t_x, t_y)
            return ini_pos, target_pos

        elif env == 2:
            choice = np.random.randint(0, 2)
            if choice == 0:
                p_x = np.random.uniform(0., .5)
                p_y = np.random.uniform(20., 20.5)
                t_x = np.random.uniform(10., 10.5)
                t_y = np.random.uniform(20., 20.5)
            else:
                p_x = np.random.uniform(10., 10.5)
                p_y = np.random.uniform(20., 20.5)
                t_x = np.random.uniform(0., .5)
                t_y = np.random.uniform(20., 20.5)

            ini_pos = {'p_x': p_x, 'p_y': p_y, 'p_z': p_z, 'o_x': o_x,
                       'o_y': o_y, 'o_z': 3.4, 'o_w': 1.5}
            target_pos = (t_x, t_y)
            return ini_pos, target_pos

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
        self.cumulated_steps = 0
        self.initial_position, self.target_p = self._get_ini_and_target_position()
        self.last_p_x = self.initial_position['p_x']
        self.last_p_y = self.initial_position['p_y']

        self.gazebo.unpauseSim()
        self.reset_position()

        time.sleep(default_sleep)
        self.gazebo.pauseSim()
        self.cumulated_steps = 0
        # plt.figure()
        # distances = self.laser_scan.ranges
        # distances = np.clip(distances, 0., 4.)
        # thetas = np.linspace(-3.14, 3.14, 897)
        # plt.polar(thetas, np.roll(distances, -224))
        plt.show()

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
        self.input_shape = (500,)
        # obs_low = np.append(np.zeros(300), np.array((-100., -100)))
        # obs_high = np.append(np.ones(300) * 10., np.array((100., 100)))
        obs_low = 0.0
        obs_high = 10.0
        # self.observation_space = spaces.Box(low=0.0, high=10.0, shape=(20,))
        self.observation_space = spaces.Tuple([
            spaces.Box(low=0., high=4., shape=(18, )),
            spaces.Box(low=-10., high=10., shape=(2,)),
            spaces.Box(low=-1., high=1., shape=(2,))
        ])
        # self.observation_space = spaces.Box(low=0.0, high=1.0, shape=(40, 128, 1))

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
        scan = self._process_scan2()
        pos_x, pos_y = self._get_pos_x_y()
        t_x, t_y = self.target_p[0], self.target_p[1]
        p_x = t_x - pos_x
        p_y = t_y - pos_y
        obs = np.append(scan, np.array([p_x, p_y]).reshape((1, 2)))

        # return (self._proc_scans(), np.clip(np.array((p_x, p_y)), 0., 9.99))
        scan = np.clip(scan.reshape((18, )), a_min=0.0, a_max=3.99)
        pos_to_target = np.clip(np.array([p_x, p_y]).reshape((2, )), a_min=-9.99, a_max=9.99)
        velocity = np.clip(np.array([self.last_p_x - p_x, self.last_p_y - p_y]).reshape((2, )), a_min=-1.0, a_max=1.0)
        self.last_p_x = p_x
        self.last_p_y = p_y
        self.target_pos_publisher.publish("t: {}, v: {}, steps: {}".format(pos_to_target, velocity, self.cumulated_steps))
        return (scan, pos_to_target, velocity)

    def _proc_scans(self):
        scan = self._process_scan()
        self.scans = np.insert(self.scans, 0, scan.reshape(1, 128), axis=0)
        self.scans = np.delete(self.scans, -1, axis=0)
        return self.scans.reshape((40, 128, 1))

    def _get_obs_old(self):
        scan = self._process_scan()
        # self.scans = np.append(self.scans, scan.reshape(1, 128), axis=0)
        # self.scans = np.delete(self.scans, 0, axis=0)
        self.scans = np.insert(self.scans, 0, scan.reshape(1, 128), axis=0)
        self.scans = np.delete(self.scans, -1, axis=0)

        if np.random.rand() < 0.1:
            plt.imsave('{}/img_logs/obs_{}.jpg'.format(Path.home(), self.obs_save_ind), self.scans)
            self.obs_save_ind += 1
        return self.scans.reshape((40, 128, 1))
        state = self._create_robot_state2()

        obs = np.zeros((48, 256))
        obs[:, :208] = self.scans
        obs[:, 208:] = state

        return obs.reshape((48, 256, 1))

    def _create_robot_state2(self):
        pos_x, pos_y = self._get_pos_x_y()

        pos_x_y = np.ones((1, 48))

        s_x = abs(self.target_p[0] - pos_x)
        s_y = abs(self.target_p[1] - pos_y)

        x_all = np.arange(-1.2, 1.2, 0.1)  # entire range of x, both in and out of spec
        # mean = 0, stddev = 1, since Z-transform was calculated
        d_x = norm.pdf(x_all, 0, s_x / 2)
        d_x = np.interp(d_x, (d_x.min(), d_x.max()), (0, 1))

        d_y = norm.pdf(x_all, 0, s_y / 2)
        d_y = np.interp(d_y, (d_y.min(), d_y.max()), (0, 1))

        d_all = np.zeros((1, 48))
        d_all[:, :24] = d_x
        d_all[:, 24:] = d_y
        self.distances = np.insert(self.distances, 0, d_all.reshape(1, 48), axis=0)
        self.distances = np.delete(self.distances, -1, axis=0)

        return self.distances.reshape(48, 48)

    def _create_robot_state(self):
        pos_x, pos_y = self._get_pos_x_y()

        def twoD_Gaussian(x, y, amplitude, xo, yo, sigma_x, sigma_y, theta, offset):
            xo = float(xo)
            yo = float(yo)
            a = (np.cos(theta) ** 2) / (2 * sigma_x ** 2) + (np.sin(theta) ** 2) / (2 * sigma_y ** 2)
            b = -(np.sin(2 * theta)) / (4 * sigma_x ** 2) + (np.sin(2 * theta)) / (4 * sigma_y ** 2)
            c = (np.sin(theta) ** 2) / (2 * sigma_x ** 2) + (np.cos(theta) ** 2) / (2 * sigma_y ** 2)
            g = offset + amplitude * np.exp(- (a * ((x - xo) ** 2) + 2 * b * (x - xo) * (y - yo)
                                               + c * ((y - yo) ** 2)))
            return g.ravel()

        x = np.linspace(0, 47, 48)
        y = np.linspace(0, 47, 48)
        x, y = np.meshgrid(x, y)

        amplitude = 1
        xo = 24
        yo = 24
        s_x = (self.target_p[0] - pos_x) * 3
        s_y = (self.target_p[1] - pos_y) * 3

        # create data
        data = twoD_Gaussian(x, y, amplitude, xo, yo, s_x, s_y, 0, 0)
        return data.reshape(48, 48)

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
        ranges = self.get_laser_scan().astype('float32') - 0.5
        # ranges = ranges[250:-250]
        ranges = np.clip(ranges, 0.0, 5.0) / 5.
        ranges_chunks = np.array_split(ranges, 128)
        ranges_mean = np.array([np.min(chunk) for chunk in ranges_chunks])
        return ranges_mean.reshape(128, )

    def _process_scan2(self):
        ranges = self.get_laser_scan().astype('float32')
        #ranges = np.roll(ranges, -224)
        #ranges = ranges[100:-100]
        ranges = np.clip(ranges, 0.0, 4.0)
        ranges_chunks = np.array_split(ranges, 18)
        ranges_mean = np.array([np.min(chunk) for chunk in ranges_chunks])
        return ranges_mean.reshape(18, )

    def _is_done(self, observations):
        self._episode_done = self._is_collided()
        if self._episode_done or self.cumulated_steps > 700:
            return True
        else:
            return False

    def _is_finished(self, obs):
        pos_x, pos_y = self._get_pos_x_y()
        d = self._get_distance((pos_x, pos_y), (self.target_p[0], self.target_p[1]))
        p_x, p_y = abs(self.target_p[0] - pos_x), abs(self.target_p[1] - pos_y)
        if d < 0.8:
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
