# Flow Planning ROS

## Install
clone
```bash
git clone --recurse-submodules -j8 git@github.com:reeceomahoney/flow_planning_ros.git
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


