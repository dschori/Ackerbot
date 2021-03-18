# Test Vehicle

![alt text](/docs/test-vehicle.png)

All software parts are running in a docker container. Either build the image from source or download it from **[docker hub](https://hub.docker.com/r/dschori/ackerbot)**.  

Note, the image does only work on ARM- Architecture!

## Building the [docker image](Dockerfile):

```
1. Clone this repository
2. On the Jetson Nano go to Ackerbot/ackerbot_real_ws/src/robot_pkg
3. Open Terminal here
4. Close all other Applications (to have enough system memory, when building on Nano)
5. sudo docker build -t dschori/ackerbot:real -f Dockerfile .
```

## Starting the docker container:

### Only the first time:

To copy the build files and directories to the local file system.

```
sudo docker run -d --name ackerbot dschori/ackerbot:real

sudo docker cp ackerbot:/workspace/catkin_ws/build ~/Ackerbot/ackerbot_real_ws/build
sudo docker cp ackerbot:/workspace/catkin_ws/devel ~/Ackerbot/ackerbot_real_ws/devel
sudo docker cp ackerbot:/workspace/catkin_ws/logs ~/Ackerbot/ackerbot_real_ws/logs  

sudo docker rm -f ackerbot  

catkin build -j2  
```
### Then:
```
sudo docker run -it --rm --privileged --net=host --name ackerbot \
-v ~/Ackerbot/ackerbot_real_ws/src/robot_pkg:/workspace/catkin_ws/src/robot_pkg \
-v ~/Ackerbot/ackerbot_real_ws/build:/workspace/catkin_ws/build \
-v ~/Ackerbot/ackerbot_real_ws/devel:/workspace/catkin_ws/devel \
-v ~/Ackerbot/ackerbot_real_ws/logs:/workspace/catkin_ws/logs \
dschori/ackerbot:real bash
```

When inside the docker container source the build with: `source devel/setup.bash`

## SLAM:

### 1. Starting the sensors launch file:

This starts the [Velodyne VLP-16](https://github.com/ros-drivers/velodyne) and the [Realsense T265](https://github.com/IntelRealSense/realsense-ros) Sensor  

Open a new Terminal and go into the running container:  
```
sudo docker exec -it ackerbot bash
```  
Inside the container type:  
```
source devel/setup.bash
roslaunch robot_pkg sensors.launch
```

### 2. Starting the mapping launch file:

For the TEB navigation as well as the localization of the vehicle [Google Cartographer](https://github.com/cartographer-project/cartographer_ros) was used as SLAM algorithm.  

To improve the mapping and localization process, IMU and Odometry Data from the Realsense T265 is fed into Cartographer.

![alt text](/docs/slam-setup.PNG)

This File starts the Google Cartographer SLAM process  
```
roslaunch robot_pkg mapping.launch
```
### 3. Driving around:
This File starts the drive controller, one can now drive around. See: [Teleop Twist](https://github.com/ros-teleop/teleop_twist_keyboard) 
```
roslaunch robot_pkg drive.launch
```

### 4. Open RVIZ

Open RVIZ with the [config file](/ackerbot_real_ws/src/robot_pkg/config/rviz_config.rviz) and monitor the nodes you want.

## TEB Navigation:

### 1. Start the Sensors as mentioned in SLAM

### 2. Starting the localization launch file:

This File localizes the robot in a given map based on the laser scans and odometry data of the T265.  

In a new Terminal run:  
```
roslaunch robot_pkg localization.launch
```
### 3. Start Rviz

Start RVIZ with the [config file](/ackerbot_real_ws/src/robot_pkg/config/rviz_config.rviz) and set a "2D Nav Goal"

## Reinforcement learning based Navigation:

### 1. Start the Sensors as mentioned in SLAM

### 2. Starting the localization launch file:

This File localizes the robot in a given map based on the laser scans and odometry data of the T265.  

In a new Terminal run:  
```
roslaunch robot_pkg localization.launch
```
### 3. Start RL Navigation

Open the [Inference Notebook](/ackerbot_sim_ws/src/rl-navigation/navigation_train/scripts/Inference.ipynb) to load the trained Agent, set a target and start the navigation.

