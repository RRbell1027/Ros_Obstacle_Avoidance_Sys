## This is a costumized ros container for Obstacle Avoidance Vehicle
As the title said, the car model "my_bot" is generated from [joshnewans/my_bot](https://github.com/joshnewans/my_bot). \
I combine the model and docker for efficiently and conveniently open it from container. \
Also, `run` provide a simple way to run ros GUI in container. (e.g. rviz2, gazebo) \

## Configuration Steps
1. Run run_docker file, it will build up image of local Dockerfile and run into bash with an enable of GUI access.
```bash
$chmod +x run
$./run
```

2. Build the workspace
```bash
$source opt/ros/iron/setup.bash
$cd /shared/workspace/
$colcon build --symlink-install
$source install/setup.bash
``` 
