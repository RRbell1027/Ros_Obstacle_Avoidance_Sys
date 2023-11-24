## This is a preworkspace docker image
Image include the clone of [joshnewans/articubot_one](https://github.com/joshnewans/articubot_one/tree/main), which is a car simumlation package with lidar, depth camera and so on sensors I need, and my obstacle avoid system.  

## Configuration Steps
1. Build the image
```bash
$docker build -t roscar_v2 .
```

2. Run the image:
Since docker cannot execute gui process solely, I provide some solution to fix the problem by this [video](https://www.youtube.com/watch?v=qWuudNxFGOQ).
* by Windows \
Install and run XLaunch before you use following command to run the container. 
```bash
$docker run --name roscar_container -e DISPLAY=host.docker.internal:0.0 -it ros_v2
```
* by Linux(Ubuntu) \
The bash file is provided in the project. Cloned by [HaiderAbasi/ROS2-Path-Planning-and-Maze-Solving](https://github.com/HaiderAbasi/ROS2-Path-Planning-and-Maze-Solving/tree/master/docker)
```bash
$chmod -x run_by_linux.bash
$./run_by_linux.bash
```

3. Build the workspace
```bash
$cd car_ws
$colcon build --symlink-install
```

4. Launch simulation
```bash
$source install/setup.bash
$ros2 launch articubot_one launch_sim.launch.py
```