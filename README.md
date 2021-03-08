# Maples Navigation: A reinforcement learning approach

**Abstrac**  
Autonomous navigation from A to B in indoor environments is a widely researched field.  However, many known approaches are based on a mapping of the entire environment in order to be able to calculate a path through this space in advance. In this Master thesis, a new approach based on reinforcement learning is proposed. With this scheme, a complete mapping of the environment is not necessary, as it depends only on current laser scan values and the position relative to the target. The method has been developed using the Robot Operating System (ROS) and Gazebo for simulation. A test vehicle with Ackermann steering was built to test the proposed method and compared  with a state of the art navigation method in a real world scenario.
Experiments have shown that the proposed method does not reach the accuracy of the existing algorithm in terms of reliability to achieve the desired goal. However, the proposed method offers potential in cases where prior mapping is not possible.

**Navigation sample using a TEB (LINK) based Planner as a baseline:**  
![alt text](docs/traj_teb.PNG)

**Navigation sample using the proposed Reinforcement Approach:**  
![alt text](docs/traj_rl.PNG)

## Documentation:

TODO

## Training in Simulation

TODO

## Test Vehicle

TODO
