-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "camera_t265_imu_optical_frame",
  published_frame = "velodyne",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4


--Local Slam
--there are more parameters to tune, but this ones are the ones I found more impactful

--this one tries to match two laser scans together to estimate the position,
--I think if not on it will rely more on wheel odometry
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

-- tune this value to the amount of samples (i think revolutions) to average over
--before estimating te position of the walls and features in the environment
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 2

--use or not use IMU, if used, the tracking_frame should be set to the one that the IMU is on
TRAJECTORY_BUILDER_2D.use_imu_data = true

--This is the scan matcher and the weights to different assumptions
--occupied_space gives more weight to the 'previous' features detected.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 20.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 100.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.

--this will help continue making the map while the robot is static
--default time is 5 seconds
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.1

--imu configuration parameters
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 0.5


--map output parameters

--Global Slam
--Setting POSE_GRAPH.optimize_every_n_nodes to 0 is a handy way
--to disable global SLAM and concentrate on the behavior of local SLAM.
--This is usually one of the first thing to do to tune Cartographer.
POSE_GRAPH.optimize_every_n_nodes = 0 --90 default
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 10
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1.
POSE_GRAPH.optimization_problem.fixed_frame_pose_translation_weight = 1e-1
POSE_GRAPH.optimization_problem.fixed_frame_pose_rotation_weight = 1e-1


return options

--TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
--TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = .1
