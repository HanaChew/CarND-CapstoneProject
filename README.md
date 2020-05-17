# CarND_Capstone
## Setup
During this session the workspace offered by Udacity is used. First, the workspace needs to be set up: 
```shell
git clone https://github.com/udacity/CarND-Capstone.git
pip install -r requirements.txt
```
The next step is to install catkin project
```shell
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
If an error `-- Could not find the required component 'dbw_mkz_msgs'. The following CMake error indicates that you either need to install the package with the same name or change your environment so that it can be found.
CMake Error at /opt/ros/kinetic/share/catkin/cmake/catkinConfig.cmake:83 (find_package):
  Could not find a package configuration file provided by "dbw_mkz_msgs" with
  any of the following names: …"` occurs, a new install of ros-kinetic-dbw-mkz-msgs will help: 
```shell
sudo apt-get update
sudo apt-get install -y ros-kinetic-dbw-mkz-msgs
cd /home/workspace/CarND-Capstone/ros
rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
```
These command lines are done within the script `/ros/update_ros_pkg`. After downloading, execute `chmod 755 update_ros_pkg`.
After updating dbw package, there could be following error `st.steering_wheel_angle_cmd = val * math.pi/180.
AttributeError: 'SteeringReport' object has no attribute 'steering_wheel_angle_cmd'`. This is because the attribute `steering_wheel_angle_cmd` got renamed to `steering_wheel_cmd` due to https://github.com/udacity/CarND-Capstone/pull/296. 
Corrected occurences in following file: 
- /ros/src/styx/bridge.py, line 104

## Pushing and Pulling to Github
### Global information
```shell
git config --global user.email "info@slieter.de"
git config --global user.name "DanielStuttgart"
```
These command lines are done within the script `/ros/init_git`.
### Initiating a new repo
```shell
git init
git add *
git commit -m "first commit"
git remote add origin https://github.com/DanielStuttgart/CarND_Capstone.git
git push -u origin master
```
### Cloning, Changing and Pushing
```shell
## Clone for first time only
git clone https://github.com/DanielStuttgart/CarND_Capstone.git
# do changes
git add *
git commit -m "some changes"
git push -u origin master
```
```shell
## Pull request
git pull origin master
# do changes
git add *
git commit -m "some changes"
git push -u origin master
```

## General overview
![](/img/final-project-ros-graph-v2.png)
Architecture overview taken from Udacity class notes

## Waypoint-Updater
- changed callback functions
- implemented waypoint updater
- added rospy.loginfo and rospy.logwarn for debugging
- changed number of waypoints from 200 to 20 due to performance reasons (see videos below)

Following video shows the performance with 200 waypoints:

![](/img/waypoint_200.gif)
- steering is not published fast enough
- even though, the received steering has another latency as well
- waypoints are not updated fast enough

In order to tackle this problem, I changed number of waypoints to 20: 
![](/img/waypoint_20.gif)

