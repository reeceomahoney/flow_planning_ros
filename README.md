# Flow Planning ROS

## Install
clone
```bash
git clone git@github.com:reeceomahoney/flow_planning_ros.git
git submodule update --init
```
build docker container
```bash
docker compose -f docker/docker-compose.yml build
docker compose -f docker/docker-compose.yml up -d
docker exec -it ros_docker bash
```
build workspace
```bash
catkin build
```

## Run
launch the simulation
```bash
roslaunch panda_gazebo panda.launch
```
launch the controller
```bash
roslaunch flow_planning controller.launch
```
