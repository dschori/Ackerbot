import time
import math
import numpy as np

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

from navigation_gym import robot_gazebo_env
from sensor_msgs.msg import LaserScan, CompressedImage
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float64, String
import matplotlib.pyplot as plt
from pathlib import Path
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from ackermann_msgs.msg import AckermannDriveStamped

from gym import spaces

import cv2

default_sleep = 1


class NavigationEnv(robot_gazebo_env.RobotGazeboEnv):
    def __init__(self):
        self.env_config = None
        self.img_size = 84

        self.initial_position = None

        # self.min_distance = .55
        self.min_distance = .3

        self.last_int_difference = 0

        self.bridge = CvBridge()

        # Doesnt have any accesibles
        self.controllers_list = []
        self.x_min, self.x_max, self.y_min, self.y_max = 0.0, 0.0, 0.0, 0.0
        self.max_area = 0.0
        # It doesnt use namespace
        self.robot_name_space = ""

        # We launch the init function of the Parent Class robot_gazebo_env.RobotGazeboEnv
        super(NavigationEnv, self).__init__(controllers_list=self.controllers_list,
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
        self.dyn1_x_min = -6.0
        self.dyn1_x_max = -2.0
        self.dyn1_last = self.dyn1_x_max
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
        self.obs_images = np.zeros((84, 84, 4))
        self.max_lidar_range = 5.
        self.observation_space = spaces.Box(low=0.0, high=1.0, shape=(self.img_size, self.img_size, 4))
        self.speed = 1.
        self.left_steering = 0.
        self.right_steering = 0.
        self.img_prefix = ''
        self.starting_pos = (0., 0.)
        self.target_pos = (0., 0.)
        self.start_orientation = (0., 0.)
        self.output_folder = ''
        rospy.logdebug("Finished NeuroRacerEnv INIT...")

    def _override_train_positions(self):
        env = np.random.randint(0, 2)
        p_x, p_y, p_z = 0.0, 0.0, 0.05
        o_x, o_y, o_z, o_w = 0.0, 0.0, 0.75, 0.75
        if env == 0:
            choice = np.random.randint(0, 2)
            if choice == 0:
                p_x = np.random.uniform(-0.5, 5.5)
                p_y = np.random.uniform(-0.5, 2.)
                if np.random.randint(0, 1) == 0:
                    t_x = np.random.uniform(-4.25, -3.75)
                    t_y = np.random.uniform(-5.25, -4.75)
                else:
                    t_x = np.random.uniform(0., 4.)
                    t_y = np.random.uniform(-5.25, -4.)
                o_w = np.random.uniform(3.8, 4.2)
            else:
                if np.random.randint(0, 1) == 0:
                    p_x = np.random.uniform(-4.25, -3.75)
                    p_y = np.random.uniform(-5.25, -4.75)
                else:
                    p_x = np.random.uniform(0., 4.)
                    p_y = np.random.uniform(-5.25, -4.)
                t_x = np.random.uniform(-0.5, 5.5)
                t_y = np.random.uniform(-0.5, 2.)
                o_w = np.random.uniform(1.3, 1.7)
            ini_pos = {'p_x': p_x, 'p_y': p_y, 'p_z': p_z, 'o_x': o_x,
                       'o_y': o_y, 'o_z': np.random.uniform(1.3, 1.7), 'o_w': o_w}
            target_pos = (t_x, t_y)
            self.target_pos = target_pos
            self.initial_position = ini_pos

        elif env == 1:
            choice = np.random.randint(0, 2)
            if choice == 0:
                p_x = np.random.uniform(12., 16.5)
                p_y = np.random.uniform(1.75, 2.25)
                t_x = np.random.uniform(12., 16.5)
                t_y = np.random.uniform(-5.75, -6.25)
                o_z, o_w = np.random.uniform(1.3, 1.7), np.random.uniform(-1.3, -1.7)
            else:
                p_x = np.random.uniform(12., 16.5)
                p_y = np.random.uniform(-5.75, -6.25)
                t_x = np.random.uniform(12.0, 16.5)
                t_y = np.random.uniform(.75, 2.25)
                o_z, o_w = np.random.uniform(3.2, 3.6), np.random.uniform(1.3, 1.7)

            ini_pos = {'p_x': p_x, 'p_y': p_y, 'p_z': p_z, 'o_x': o_x,
                       'o_y': o_y, 'o_z': o_z, 'o_w': o_w}
            target_pos = (t_x, t_y)
            self.target_pos = target_pos
            self.initial_position = ini_pos

    def _get_ini_and_target_position(self):

        p_x, p_y, p_z = 0.0, 0.0, 0.05
        o_x, o_y, o_z, o_w = 0.0, 0.0, 0.75, 0.75
        p_x, p_y = self.starting_pos[0] + np.random.uniform(-0.2, 0.2), self.starting_pos[1] + np.random.uniform(-0.2,
                                                                                                                 0.2)
        t_x, t_y = self.target_pos[0], self.target_pos[1]
        o_z, o_w = self.start_orientation[0], self.start_orientation[1]
        ini_pos = {'p_x': p_x, 'p_y': p_y, 'p_z': p_z, 'o_x': o_x,
                   'o_y': o_y, 'o_z': o_z, 'o_w': o_w}
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
        super(NavigationEnv, self).reset()
        self.obs_images = np.zeros((84, 84, 4))
        self.cumulated_steps = 0
        #self.initial_position, self.target_p = self._get_ini_and_target_position()
        self._override_train_positions()
        self.last_p_x = self.initial_position['p_x']
        self.last_p_y = self.initial_position['p_y']

        self.gazebo.unpauseSim()
        self.reset_position()

        time.sleep(default_sleep)
        self.gazebo.pauseSim()
        self.cumulated_steps = 0
        self.speed = np.random.uniform(0.2, 1.0)
        self.left_steering = np.random.uniform(0.35, 1.0)
        self.right_steering = np.random.uniform(-0.35, -1.0)
        self.speed = 0.6

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
        self.odom = None
        rospy.logdebug("Waiting for ground_truth/state to be READY...")
        while self.odom is None and not rospy.is_shutdown():
            try:
                self.odom = rospy.wait_for_message('ground_truth/state',
                                                   Odometry,
                                                   timeout=1.0)
            except:
                rospy.logerr("Odom not ready yet, retrying for getting odom_msg")

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
        if self.output_folder != '':
            output_folder = '{}/img_logs'.format(self.output_folder)
            Path(output_folder).mkdir(parents=True, exist_ok=True)
            plt.imsave('{}/{}_obs_{}.png'.format(output_folder, self.img_prefix, self.obs_save_ind + 1000000), image,
                       cmap='viridis')
        return image

    def _process_scan(self):
        ranges = self.get_laser_scan().astype('float32') - 0.1
        ranges = np.clip(ranges, 0.0, 8.0) / 8.
        ranges_chunks = np.array_split(ranges, 20)
        ranges_mean = np.array([np.min(chunk) for chunk in ranges_chunks])
        return ranges_mean.reshape(20, )

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

    def get_odom(self):
        return self.odom

    def _get_orientation(self):
        odom = self.get_odom()
        ox = odom.pose.pose.orientation.x
        oy = odom.pose.pose.orientation.y
        oz = odom.pose.pose.orientation.z
        ow = odom.pose.pose.orientation.w
        return (ox, oy, oz, ow)

    def _get_distance(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def _get_angle_z(self):
        odom = self.get_odom()
        orientation_list = [odom.pose.pose.orientation.x,
                            odom.pose.pose.orientation.y,
                            odom.pose.pose.orientation.z,
                            odom.pose.pose.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        return yaw

    def _get_pos_x_y(self):
        odom = self.get_odom()
        px = odom.pose.pose.position.x
        py = odom.pose.pose.position.y
        return (px, py)

    def _is_done(self, observations):
        self._episode_done = self._is_collided()
        if self._episode_done or self.cumulated_steps > 400:
            # if self._episode_done or self.cumulated_steps > 500:
            return True
        else:
            return False

    def _is_finished(self, obs):
        pos_x, pos_y = self._get_pos_x_y()
        d = self._get_distance((pos_x, pos_y), (self.target_p[0], self.target_p[1]))
        p_x, p_y = abs(self.target_p[0] - pos_x), abs(self.target_p[1] - pos_y)
        # if d < 0.8:
        if d < 0.5:
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
        ranges = np.clip(ranges, 0.0, max_lidar_range)

        # ranges_chungs = np.array_split(ranges, 72)
        # ranges = [np.ones((len(chunk),)) * np.min(chunk) for chunk in ranges_chungs]
        # ranges = np.concatenate(ranges)

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
                # set all pixels to white until b_i is equal to current range
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

        return blank_image
