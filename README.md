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
- ´rostopic list` shows /final_waypoints
- `rostopic info /final_waypoints` shows correct type and architecture
- `rostopic echo /final_waypoints` does not show any waypoints 
- pose is not called and set
