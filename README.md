# Robotics-lab-project

Repository of robotics laboratory project 

### Description
project based on ROS, <br>
the goal of this project is recognize some mega blocks bricks using zed2 sterocam, and then move the arm robot to take the bricks and move to another position<br>
framework used: ROS-noetic (https://wiki.ros.org/noetic)

### Installation and Requirements <br>
first of all, download and install ros: <br>
following this tutorial [link](https://github.com/mfocchi/locosim) <br>
install eigen library
```bash
    sudo apt install libeigen3-dev
```
then clone this repository:
```bash
git clone https://github.com/MrLakige/robot_project
cd robot_project
```

### Usage
create a catkin workspace 
```bash
catkin init
```
clone the repository and compile
```bash
catkin_make install
```
add source 
```bash
source devel/setup.bash
```


## Authors and acknowledgment
MrLakige HappaBaboo Indra999

## Project status
work in progress
