Apriltag Toolbox
================

![ROS](https://img.shields.io/badge/ros-%230A0FF9.svg?style=for-the-badge&logo=ros&logoColor=white)

![workflow](https://github.com/kousuke-takeuchi/apriltag-toolbox/actions/workflows/test.yml/badge.svg)

### installation

```sh
sudo apt install python-catkin-tools 
sudo apt -y remove gtsam
sudo add-apt-repository -y --remove ppa:bernd-pfrommer/gtsam
sudo apt-add-repository -y ppa:borglab/gtsam-release-4.1
sudo apt update
sudo apt install libgtsam-dev libgtsam-unstable-dev 
```
