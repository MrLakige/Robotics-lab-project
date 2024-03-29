# Robotics-lab-project

Repository of robotics laboratory project 

## Description
Project based on ROS, <br>
The goal of this project is recognize certain Mega Bloks bricks using zed2 stereo camera, and then move the robotic arm to take the bricks and move to another position<br>
framework used: [ROS-noetic](https://wiki.ros.org/noetic)<br>
#### Documentation
Project documentation [here](https://mrlakige.github.io/Robotics-lab-project/)
___
## Video

[![Watch the video](https://i3.ytimg.com/vi/Jikigpxwn9s/maxresdefault.jpg)](https://www.youtube.com/watch?v=Jikigpxwn9s)
___
## Installation and Requirements <br>
Download and install ROS [(guide here)](https://github.com/mfocchi/locosim) <br>
#### Kinematics part
Install `eigen` library
```bash
sudo apt install libeigen3-dev
```
#### Vision part
- Install `cuda` [(guide here)](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html)
- Install `yolov5`
```bash
pip install yolov5
```

then clone this repository:
```bash
git clone https://github.com/MrLakige/robot_project
cd robot_project
```
___
## Usage
#### Set up local enviroment
Create a catkin workspace 
```bash
catkin init
```
Clone the repository and compile
```bash
catkin_make install
```
Add source 
```bash
source devel/setup.bash
```
### Run the project
1. Launch simulation enviroment
    ```bash
    cd src/locosim/robot_control/lab_exercises/lab_palopoli
    python3 ur5_generic.py
    ```
2. Bricks spawn, you can choose the difficult type level<br>
   LEVELS: <br>
   Level 1 
   ```bash
   rosrun spawner spawner1.py
   ```
    Level 2
    ```bash
    rosrun spawner spawner2.py
    ```
    Level 3
    ```bash
    rosrun spawner spawner3.py
    ```
    Level 4
    ```bash
    rosrun spawner spawner4.py
    ```
4. Run kinematics, are available 4 levels are available<br>
    LEVELS: <br>
    Level 1
   ```bash
   rosrun kinematics kinematics 1
   ```
    Level 2
   ```bash
   rosrun kinematics kinematics 2
   ```
    Level 3
   ```bash
   rosrun kinematics kinematics 3
   ```
    Level 4
   ```bash
   rosrun kinematics kinematics 4
   ```
5. Run bricks vision recognition<br>
    without graphical output
    ```bash
    rosrun vision vision_node.py
    ```
    with graphical output
    ```bash
    rosrun vision vision_node.py -show
    ```
