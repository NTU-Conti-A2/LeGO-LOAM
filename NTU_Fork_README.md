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
    - 

## Known Issues

- utility.h will include pcl header files that fail to compile due to a complaint about Eigen::Index not being found
    -  see https://github.com/RobustFieldAutonomyLab/LeGO-LOAM/issues/215 for the voxel_grid.h edit fix, and other proposed solutions
    - currently, we do in fact just replace references to Eigen::Index with int
    - /usr/include/pcl-1.10/pcl/filters/voxel_grid.h
    - this is probably a result of the Gtsam dependency including a modified version of Eigen which is too old to define this member, since the system Eigen should be high enough
- libmetis.so
    - for some reason, the system may fail to find this library
    - fixed by 
        ``` 
        apt install libparmetis-dev 
        ```
- tf2 compatibility
    - LeGO-LOAM hardcodes a lot of frame names with leading backslashes. This is illegal in tf2.



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