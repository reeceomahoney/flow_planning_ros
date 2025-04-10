# Flow Planning ROS

## Install
```bash
git clone --recurse-submodules -j8 git://github.com/reeceomahoney/flow_planning_ros.git

cd src/orocos_kinematics_dynamics
git checkout b35c424e
cd ../..

docker compose -f docker/docker-compose.yml build
docker compose -f docker/docker-compose.yml up -d
docker exec -it ros_docker bash

catkin build
```


