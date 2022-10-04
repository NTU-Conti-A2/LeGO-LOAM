# README

This fork is an attempt to get LeGO-LOAM to compile and run in a docker container based on ROS Noetic and Ubuntu 20.04 (Focal). See NTU_Fork_README.md for more info on this.

## Usage with TM

If using this repo with the TM code https://github.com/NTU-Conti-A2/topological-mapping-and-navigation please see the README files in that repo

basically it should work out of the box as long as you first execute (inside your docker container or working environment)

    ```  
    mkdir ~/Downloads
    wget -O ~/Downloads/gtsam.zip https://github.com/borglab/gtsam/archive/refs/tags/4.0.3.zip
    cd ~/Downloads/ && unzip gtsam.zip -d ~/Downloads/
    cd ~/Downloads/gtsam-4.0.3/
    mkdir build && cd build

    apt install gcc-10 g++-10
    export CC=gcc-10
    export CXX=g++-10
    cmake ..
    make install
    ```
and then apt install libparmetis-dev
- this takes care of the LeGO-LOAM dependencies such as gtsam (which needs to be compiled first before you compile LeGO-LOAM)



## Standalone Usage via Docker

If you just want to use the LeGO-LOAM algorithm with the Corriere Robot rosbag


Make sure your system has **docker** and **docker-compose** installed.
### 1) Build ntu-lego-loam image
Navigate to the parent directory of the repo run:

    docker build -t ntu-lego-loam:latest -f docker/Dockerfile .

### 2) Run the ntu-lego-loam container
Navigate into the docker folder and run docker-compose up -d:

    cd docker
    docker-compose up -d

### 3) Run the LeGo-LOAM launch files
Open a shell instance of the container using:

    docker exec -it docker_dev_lego-loam_1 zsh
Run the Lego-LOAM launch file using:

    roslaunch lego_loam run.launch
To open your local rviz with preset settings using:
    
    cd ../LeGO-LOAM/launch
    rosrun rviz rviz -d test.rviz


## Usage

- Map Cloud
    - /laser_cloud_surround
    - frame_id: camera_init
    - hz: 0.2
- Map Cloud (stack)
    - /registered_cloud
    - frame_id: camera_init
    - hz: 2.5
- Trajectory
    - /key_pose_origin
    - hz: 2.5
    - this is a point cloud which actually represents the fully corrected odometry of the robot. That is, loop closure will correct this
    - frame_id: camera_init

## Directory Structure

**LeGO-LOAM** (Parent)<br>
├── LeGO-LOAM<br>
│   ├── include<br>
│   ├── launch<br>
│   └── src<br>
├── cloud_msgs<br>
│   └── msg<br>
└── docker<br>