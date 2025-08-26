# cy_robot_dynamics_example

1. Enter the ROS2 workspace folder

```
cd ~/orca_app/ros2_ws/
```

2. Start the dynamics node

```
ros2 launch cy_robot_dynamics robot_dynamics.launch.py
```

3. Start the example nodes

```
ros2 run cy_robot_dynamics_example left_example # left hand
ros2 run cy_robot_dynamics_example right_example # right hand
```

4. The following information is displayed in the terminal

![](./assets/picture01.png)
