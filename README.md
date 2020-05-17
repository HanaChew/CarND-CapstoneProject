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
Corrected occurences in following files: 
- /ros/src/twist_controller/dbw_test.py, line 81
- /ros/src/styx/bridge.py
- /ros/src/twist_controller/dbw_node.py

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

## Waypoint-Updater
- changed callback functions
- added rospy.login for debugging
  - logging to `/root/.ros/log/8c8d45f6-9811-11ea-8784-0242ac110002/`
- `rostopic list` shows /final_waypoints
- `rostopic info /final_waypoints` shows correct type and architecture
- `rostopic echo /final_waypoints` does not show any waypoints 
- current_pose is not published, callback not called and internal variable pose is not set
  - `rostopic echo /current_pose` results in WARNING: topic does not appear to be published yet
  - dependent on order of execution
  1. start simulator
  2. start ros launcher
