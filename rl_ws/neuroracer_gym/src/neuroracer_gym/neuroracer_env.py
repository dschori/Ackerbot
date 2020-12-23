import time
import math

import numpy as np

import rospy
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState

from openai_ros import robot_gazebo_env
from sensor_msgs.msg import LaserScan, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
# from std_msgs.msg import Float64
# from sensor_msgs.msg import Image
# from tf.transformations import quaternion_from_euler

from ackermann_msgs.msg import AckermannDriveStamped

from gym import spaces
from gym.envs.registration import register

import cv2

default_sleep = 1

class NeuroRacerEnv(robot_gazebo_env.RobotGazeboEnv):
    def __init__(self):
        
        self.initial_position = None
        
        self.min_distance = .255

        self.bridge = CvBridge()

        # Doesnt have any accesibles
        self.controllers_list = []

        # It doesnt use namespace
        self.robot_name_space = ""

        # We launch the init function of the Parent Class robot_gazebo_env.RobotGazeboEnv
        super(NeuroRacerEnv, self).__init__(controllers_list=self.controllers_list,
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

        #self.controllers_object.reset_controllers()
        self._check_all_sensors_ready()
        
        self._init_camera()

        self.laser_subscription = rospy.Subscriber("/scan", LaserScan, self._laser_scan_callback)
        
        self.drive_control_publisher= rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation",
                                                       AckermannDriveStamped,
                                                       queue_size=20)

        self._check_publishers_connection()

        self.gazebo.pauseSim()
        
        rospy.logdebug("Finished NeuroRacerEnv INIT...")

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
        super(NeuroRacerEnv, self).reset()
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
        rospy.logdebug("ALL SENSORS READY")

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
        self.input_shape = img.shape
        obs_low = 0
        obs_high = 1
        self.observation_space = spaces.Box(low=obs_low, high=obs_high, shape=self.input_shape)

        img_dims = img.shape[0]*img.shape[1]*img.shape[2]
        byte_size = 4
        overhaead = 2 # reserving memory for ros header
        buff_size = img_dims*byte_size*overhaead
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
        return self.get_camera_image()

    def _is_done(self, observations):
        self._episode_done = self._is_collided()
        return self._episode_done
        
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
    
    def laserscan_to_image(self, scan):
        # Discretization Size
        disc_size = .16
        # Discretization Factor
        disc_factor = 1/disc_size
        # Max Lidar Range
        max_lidar_range = 10
        # Create Image Size Using Range and Discretization Factor
        image_size = int(max_lidar_range*2*disc_factor)
        
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
        # Calculate the number of points in array of ranges
        num_pts = len(ranges)
        # Create Array for extracting X,Y points of each data point
        xy_scan = np.zeros((num_pts,2))
        # Create 3 Channel Blank Image
        blank_image = np.zeros((image_size,image_size,3),dtype=np.uint8)
        # Loop through all points converting distance and angle to X,Y point
        for i in range(num_pts):
            # Check that distance is not longer than it should be
            if (ranges[i] > 10) or (math.isnan(ranges[i])):
                pass
            else:
                # Calculate angle of point and calculate X,Y position
                angle = minAngle + float(i)*angleInc
                xy_scan[i][0] = float(ranges[i]*math.cos(angle))
                xy_scan[i][1] = float(ranges[i]*math.sin(angle))

        # Loop through all points plot in blank_image
        for i in range(num_pts):
            pt_x = xy_scan[i,0]
            pt_y = xy_scan[i,1]
            if (pt_x < max_lidar_range) or (pt_x > -1 * (max_lidar_range-disc_size)) or (pt_y < max_lidar_range) or (pt_y > -1 * (max_lidar_range-disc_size)):
                pix_x = int(math.floor((pt_x + max_lidar_range) * disc_factor))
                pix_y = int(math.floor((max_lidar_range - pt_y) * disc_factor))
                if (pix_x > image_size) or (pix_y > image_size):
                    print("Error")
                else:
                    blank_image[pix_y,pix_x] = [0,0,255]

        # Convert CV2 Image to ROS Message
        img = self.bridge.cv2_to_imgmsg(blank_image, encoding="bgr8")
        # Publish image
        return blank_image
    
        
    def get_laser_scan(self):
        laser_scan = np.array(self.laser_scan.ranges, dtype=np.float32)
        return laser_scan
    
    def get_camera_image(self):
        try:
            #cv_image = self.bridge.compressed_imgmsg_to_cv2(self.camera_msg).astype('float32')
            cv_image = self.laserscan_to_image(self.laser_scan).astype('float32')
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
