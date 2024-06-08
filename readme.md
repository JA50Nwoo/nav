基于：
https://github.com/hikashi/multi-robot-rrt-exploration-melodic
修改，基于单机器人探索，增加了：
1. 自适应探索速率和自适应采样步长；
2. 结合人工势场法的RRT采样；
3. 改进后的代价函数和成本函数。

与原版本相比，不需要额外点四个点确认边界而是直接开始，指令：
安装18.04库：
sudo apt-get install ros-melodic-gmapping
sudo apt-get install ros-melodic-navigation
sudo apt-get install python-opencv
sudo apt-get install python-numpy
sudo apt-get install python-scikits-learn
sudo apt-get install ros-melodic-teb-local-planner

安装包：
sudo mkdir -p ~/catkin_explore/src
cd ~/catkin_explore/src/
git clone https://github.com/hikashi/multi-robot-rrt-exploration-melodic.git
cd ~/catkin_explore
catkin_make
单个机器人：
T1：
roscore 
T2：
source ~/catkin_explore/devel/setup.bash 
export TURTLEBOT3_MODEL=waffle_pi
roslaunch ros_multitb3 single_tb3_house.launch
roslaunch ros_multitb3 single_tb3_amz_smallhouse.launch
T3：
source ~/catkin_explore/devel/setup.bash 
export TURTLEBOT3_MODEL=waffle_pi
roslaunch rrt_exploration single_tb3_exploration.launch

系统版本：
rosdistro: melodic
rosversion: 1.14.13
Linux version 5.4.0-152-generic (buildd@lcy02-amd64-051) (gcc version 7.5.0 (Ubuntu 7.5.0-3ubuntu1~18.04)) #169~18.04.1-Ubuntu SMP Wed Jun 7 22:22:24 UTC 2023
Python 2.7.17
gazebo版本version 9.0.0

