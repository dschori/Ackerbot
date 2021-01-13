## Building the docker image:

```
sudo docker build -t ros-noetic-realsense -f Dockerfile.ros.noetic .
```

## Starting the docker container:

```
sudo docker run -it --privileged --net=host --name weed_bot \
-v /home/dschori/MasterThesis/catkin_ws/src/robot_pkg:/workspace/catkin_ws/src/robot_pkg \
-v /home/dschori/MasterThesis/catkin_ws/build:/workspace/catkin_ws/build \
-v /home/dschori/MasterThesis/catkin_ws/devel:/workspace/catkin_ws/devel \
-v /home/dschori/MasterThesis/catkin_ws/logs:/workspace/catkin_ws/logs \
ros-noetic-realsense bash
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
```
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
