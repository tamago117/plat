# plat
[Tsukuba Challenge](https://tsukubachallenge.jp/2022/) running robot

- basic repository for autonomous mobile robot -> [eg_navigation](https://github.com/tamago117/eg_navigation)
- micro computer program -> [plat_driver](https://github.com/tamago117/plat_driver)

![plat](https://user-images.githubusercontent.com/38370926/207793427-3c513ad4-c197-48a4-8a40-d4c02b626113.png)

### [youtube link](https://www.youtube.com/watch?v=ahRZplaeSao)

# Requirments

- ROS noetic
- Gazebo(ver9.4.0) (for simulation) ->[update gazebo](https://bitbucket.org/DataspeedInc/velodyne_simulator/src/master/gazebo_upgrade.md)

# install
```
cd ~/catkin_ws/src
git clone https://github.com/tamago117/plat
cd ..

sudo apt update
sudo apt-get install python3-rosdep
sudo apt install python3-vcstool

vcs import src < src/plat/.rosinstall --recursive
rosdep install -i --from-paths src/plat
catkin build
source ~/catkin_ws/devel/setup.bash
```