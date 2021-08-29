# Python3 Implementation of F1Tenth

* Ubuntu 20.04
* ROS Noetic

For more information, please refer to https://f1tenth.org/

## Usage
1) Clone this github repository
```
git clone https://github.com/cyuanqi920/F1Tenth
```
2) Create a workspace
```
mkdir -p ~/catkin_ws/src
```
3) Setup the F1Tenth Simulator by following instructions in https://github.com/f1tenth/f1tenth_simulator

4) Link the respective package to your workspace
```
sudo ln -fs /path_to_this_repo/lab1 ~/catkin_ws/src/lab1
```
5) Setup
```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

6) Launch the package respectively
Python
```
roslaunch lab1 lab1-py.launch
```
C++
```
roslaunch lab1 lab1-cpp.launch
```

