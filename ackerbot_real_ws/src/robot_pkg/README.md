## Building the docker image:

```
1. On the robot go to MasterThesis/ackerbot_real_ws/src/
2. Open Terminal here
3. Close all other Applications (because of enough system memory)
4. sudo docker build -t ackerbot-real -f Dockerfile .
```

## Starting the docker container:

```
sudo docker run -it --privileged --net=host --name weed_bot \
-v /home/dschori/MasterThesis/catkin_ws/src/robot_pkg:/workspace/catkin_ws/src/robot_pkg \
-v /home/dschori/MasterThesis/catkin_ws/build:/workspace/catkin_ws/build \
-v /home/dschori/MasterThesis/catkin_ws/devel:/workspace/catkin_ws/devel \
-v /home/dschori/MasterThesis/catkin_ws/logs:/workspace/catkin_ws/logs \
ackerbot-real bash
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
