## This is a costumized ros container for Obstacle Avoidance Vehicle
As the title said, the car model "my_bot" is generated from [joshnewans/my_bot](https://github.com/joshnewans/my_bot).
I combine the model and docker for efficiently and conveniently open it from container.
Also, run_docker.bash provide a simple way to run ros GUI in container. (e.g. rviz2, gazebo)
But I still have to source setup.bash and build the workspace by myself. Maybe I will fix it at the future.

## Configuration Steps
1. Run run_docker file, it will build up image of local Dockerfile and run into bash with an enable of GUI access.
```bash
$chmod +x *
$./run_docker.bash
```

2. Build the workspace
```bash
$source opt/ros/iron/setup.bash
$cd car_ws/
$colcon build --symlink-install
``` 

3. Launch rsp.launch.py file, and the car model will be ready.
```bash
$source install/setup.bash
$ros2 launch my_bot rsp.launch.py
```

## Container tree
.
├── car_ws
│   └── src
│       └── my_bot
│           ├── CMakeLists.txt
│           ├── config
│           │   ├── empty.yaml
│           │   └── veiw_bot.rviz
│           ├── description
│           │   ├── inertial_macros.xacro
│           │   ├── robot.urdf.xacro
│           │   └── robot_core.xacro
│           ├── launch
│           │   └── rsp.launch.py
│           ├── package.xml
│           └── worlds
│               └── empty.world
└── run_docker.bash
