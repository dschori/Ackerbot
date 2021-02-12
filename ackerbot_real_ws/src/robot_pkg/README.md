## Building the docker image:

```
1. On the robot go to MasterThesis/ackerbot_real_ws/src/
2. Open Terminal here
3. Close all other Applications (to have enough system memory)
4. sudo docker build -t dschori/ackerbot:real -f Dockerfile .
```

zuerst rocore auf laptop
dann diese im docker:
export ROS_MASTER_URI="http://192.168.178.50:11311"
export ROS_MASTER_URI="http://10.42.0.143:11311"

export ROS_MASTER_URI="http://192.168.33.3:11311"

rosbag record -a -O data.bag -x "/camera_t265/fisheye1/(.*)|/camera_t265/fisheye2/(.*)" --split --size 1024


## Starting the docker container:

### Only the first time:

```
sudo docker run -d --name ackerbot dschori/ackerbot:real

sudo docker cp ackerbot:/workspace/catkin_ws/build ~/MasterThesis/ackerbot_real_ws/build
sudo docker cp ackerbot:/workspace/catkin_ws/devel ~/MasterThesis/ackerbot_real_ws/devel
sudo docker cp ackerbot:/workspace/catkin_ws/logs ~/MasterThesis/ackerbot_real_ws/logs  

sudo docker rm -f ackerbot  

catkin build -j2  
```
### Then:
```
sudo docker run -it --rm --privileged --net=host --name ackerbot \
-v ~/MasterThesis/ackerbot_real_ws/src/robot_pkg:/workspace/catkin_ws/src/robot_pkg \
-v ~/MasterThesis/ackerbot_real_ws/build:/workspace/catkin_ws/build \
-v ~/MasterThesis/ackerbot_real_ws/devel:/workspace/catkin_ws/devel \
-v ~/MasterThesis/ackerbot_real_ws/logs:/workspace/catkin_ws/logs \
dschori/ackerbot:real bash
```

### Building and sourcing
`
go to: ~/catkin_ws
`  
`
then: catkin build  
`  
`
then: source/devel/setup.bash  
`  

### Starting the sensors launch file:

Open a new Terminal and go into the running container:  
```
sudo docker exec -it ackerbot bash
```  
Inside the container type:  
```
source devel/setup.bash
roslaunch robot_pkg sensors.launch
```

### Starting the mapping launch file:
```
roslaunch robot_pkg mapping.launch
```

#### Driving around:
```
roslaunch robot_pkg drive.launch
```

#### View Map:
1. Start Rviz
2. Open Rviz Config File
