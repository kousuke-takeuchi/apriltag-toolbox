Apriltag Toolbox
================

![ROS](https://img.shields.io/badge/ros-%230A0FF9.svg?style=for-the-badge&logo=ros&logoColor=white)

[![Test action-ros-ci](https://github.com/kousuke-takeuchi/apriltag-toolbox/actions/workflows/test.yml/badge.svg)](https://github.com/kousuke-takeuchi/apriltag-toolbox/actions/workflows/test.yml)

### installation

##### GTSAM 4.1

```sh 
sudo apt -y remove gtsam
sudo add-apt-repository -y --remove ppa:bernd-pfrommer/gtsam
sudo apt-add-repository -y ppa:borglab/gtsam-release-4.1
sudo apt update
sudo apt install libgtsam-dev libgtsam-unstable-dev 
```

##### (ros melodic) create workspace for python3

```bash
mkdir -p ~/secondary_build_ws/src && cd ~/secondary_build_ws
catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so && catkin config --install
git clone -b melodic-devel https://github.com/ros/geometry2 src/geometry2

catkin build

pip install empy
pip3 install empy

source /opt/ros/melodic/setup.bash
source ~/secondary_build_ws/devel/setup.bash --extend
export PYTHONPATH=~/secondary_build_ws/devel/lib/python3/dist-packages:$PYTHONPATH
```

##### 3rd party ros packages

```sh
rosdep install --from-paths src --ignore-src -r -y

pip3 install gtsam pupil-apriltags

source ~/catkin_ws/devel/setup.bash
```