Install ROS-melodic full desktop

```bash
# 18.04 ros
sudo apt install curl -y

sudo sh -c 'echo "deb <http://packages.ros.org/ros/ubuntu> $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s <https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc> | sudo apt-key add -
sudo apt-get update && sudo apt-get install dpkg
sudo apt-get install ros-melodic-desktop-full -y

echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt install python3-rosdep -y

sudo rosdep init
rosdep update
```

Install the grid_map package that can be optionally used for mapping

```bash
## 
sudo apt-get install ros-melodic-grid-map -y
```

Install ROS-Indigo move base package

```bash
##
sudo apt-get install ros-melodic-navigation -y
```



下载Gazebo模型：

```bash
mkdir -p ~/.gazebo/models
cd ~/.gazebo/models/
wget <http://file.ncnynl.com/ros/gazebo_models.txt>
wget -i gazebo_models.txt
ls model.tar.g* | xargs -n1 tar xzvf
```

克隆项目：

```bash
cd $HUSKY_DIR
git clone https://github.com/LogicT5/Husky_Simulation.git src
```

修改完成后编译测试：

```bash
# 检查安装需要的包
cd $HUSKY_DIR
rosdep install --from-path src --ignore-src --rosdistro=melodic -y

# 编译工作空间
catkin_make -DCMAKE_BUILD_TYPE=Release

echo "source $HUSKY_DIR/devel/setup.bash" >> ~/.bashrc
echo "export HUSKY_GAZEBO_DESCRIPTION=$(rospack find husky_gazebo)/urdf/description.gazebo.xac" >> ~/.bashrc
source ~/.bashrc
```

launches customized husky gazebo (`gazebo`).

```bash
roslaunch husky_gazebo husky_playpen.launch
```

注意Husky模型是否完整。

launches customized husky visualization (`rviz`).

```bash
roslaunch husky_viz view_robot.launch
```

LOAM can be used to SLAM (mapping and location robot) as follow(gp-insac algorithm is launched parallelly, which is to devide ground, obstacle and boundary points):

```bash
roslaunch loam_velodyne loam_velodyne.launch
```

Real-time reconstruction of a single frame

```bash
roslaunch frame_reconstruction reconstruction.launch
```

For mapping with topological and geometrical information. It should be noted that this node has a countdown of ten seconds. If it still have not received the data from LOAM node after the countdown, it will be automatically terminated.

```bash
roslaunch topo_confidence_map mapping.launch
```

To automatically send the goal of mapping results to husky moving action

```bash
roslaunch husky_navigation_goals send_goal.launch
```
