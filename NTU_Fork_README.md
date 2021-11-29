This fork is an attempt to get LeGO-LOAM to compile and run in a docker container based on ROS Noetic and Ubuntu 20.04 (Focal).

## Notable Changes

- utility.h has the opencv header file #Include updated to 
    ```
    #include <opencv2/opencv.hpp> // update opencv version
    ```
- CMakeLists.txt
    - c++ standard is updated from 11 to 14, to accomodate PCL 1.10
    - Boost library components are added to the CMakeLists.txt, since they were failing to link
        ```
        find_package(Boost REQUIRED COMPONENTS timer thread serialization chrono) 
        ```
    - as per https://github.com/RobustFieldAutonomyLab/LeGO-LOAM/issues/224  
- frame ids (removing leading forwardslash)
    - changed run.launch file 
        - /map
        - /camera_init
        - /base_link

## Known Issues

- utility.h will include pcl header files that fail to compile due to a complaint about Eigen::Index not being found
    -  see https://github.com/RobustFieldAutonomyLab/LeGO-LOAM/issues/215 
        - for the voxel_grid.h edit fix, and other proposed solutions
    - currently, we do in fact just replace references to Eigen::Index with int
    - /usr/include/pcl-1.10/pcl/filters/voxel_grid.h
    - this is probably a result of the Gtsam dependency including a modified version of Eigen which is too old to define this member, since the system Eigen should be high enough
- libmetis.so
    - for some reason, the system may fail to find this library
    - fixed by 
        ``` 
        apt install libparmetis-dev 
        ```
    - as per https://github.com/RobustFieldAutonomyLab/LeGO-LOAM/issues/160 
- tf2 compatibility
    - LeGO-LOAM hardcodes a lot of frame names with leading forward slashes. This is illegal in tf2.
    - removed them all from the launch and cpp files. remove these from frame_id variables, not ros topic names.


## Changes for Corriere

- the LIDAR topic/frame
    - /rslidar_points with the frame_id: "rslidar"
    - change utility.h
        - pointCloudTopic = "/rslidar_points"
        - useCloudRing = false;
        - parameters
            - Horizon_SCAN = 3600
            - ang_res_x = 0.18
        - sensorMinimumRange was left as the original 1.0
- imu seems the same 
- dawei used https://github.com/HTLife/lego_loam_docker which has some changes already made.
    - why was sensorMinimumRange removed? Actually, a bunch of stuff was deleted for no clear reason. 
        - delete the PointXYZIR struct? and delete the corresponding macro?
            - this seems unnecessary, since it's not used anywhere if you just set the flag to false
            - most point clouds are of type PointType which is typedef in utility.h
- the main issue is adjusting to the lack of a ring channel in the lidar points
    - it appears that LeGO-LOAM was originally designed for non ring channel points, so this isn't too hard

- enabled loop closure

## Misc

Might be useful

```
set(CMAKE_CXX_STANDARD 14) 

set(CMAKE_CXX_STANDARD_REQUIRED ON) 

```

### Potential Changes
- gtsam has newer versions. 
    - Using these instead could potentially fix some things like the voxel_grid.h problem
- gtsam CMakeLists.txt should have an option to force usage of system Eigen instead of their modified Eigen. 
    - need to investigate what features would be lost. See their readme.


## LeGO-LOAM Sample Bag

- publishes two topics
- /velodyne_points 
    - frame_id: "velodyne"
- /imu/data
    - frame_id: "imu_link"


