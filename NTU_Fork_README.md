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
    - also need to change /aft_mapped and /aft_mapped_to_init
    

## Known Issues

- utility.h will include pcl header files that fail to compile due to a complaint about Eigen::Index not being found
    -  see https://github.com/RobustFieldAutonomyLab/LeGO-LOAM/issues/215 
        - for the voxel_grid.h edit fix, and other proposed solutions
    - currently, we do in fact just replace references to Eigen::Index with int
    - /usr/include/pcl-1.10/pcl/filters/voxel_grid.h
    - this is probably a result of the Gtsam dependency including a modified version of Eigen which is too old to define this member, since the system Eigen should be high enough
        - the suggested version of gtsam is 4.0.0-alpha2 which was released Sep 24 2017 https://github.com/borglab/gtsam/releases/tag/4.0.0-alpha2
        - based on commit history at https://github.com/borglab/gtsam/commits/develop/gtsam/3rdparty/Eigen , this used Eigen 3.2.10
            - can also check the output of running 'cmake ..' when building gtsam, which also shows the 'Use System Eigen' flag status.
        - the next update was Oct 10 2018, to Eigen 3.3.4 , which should be sufficient to resolve the Eigen::Index compilation error.

            ```
            [ 73%] Building CXX object LeGO-LOAM/LeGO-LOAM/CMakeFiles/featureAssociation.dir/src/featureAssociation.cpp.o
            cd /root/catkin_ws/build/LeGO-LOAM/LeGO-LOAM && /usr/bin/c++  -DDISABLE_LIBUSB_1_0 -DDISABLE_PCAP -DDISABLE_PNG -DROSCONSOLE_BACKEND_LOG4CXX -DROS_BUILD_SHARED_LIBS=1 -DROS_PACKAGE_NAME=\"lego_loam\" -Dqh_QHpointer -DvtkRenderingContext2D_AUTOINIT="1(vtkRenderingContextOpenGL2)" -DvtkRenderingCore_AUTOINIT="3(vtkInteractionStyle,vtkRenderingFreeType,vtkRenderingOpenGL2)" -I/root/catkin_ws/src/LeGO-LOAM/LeGO-LOAM/include -I/root/catkin_ws/devel/include -I/opt/ros/noetic/include -I/opt/ros/noetic/share/xmlrpcpp/cmake/../../../include/xmlrpcpp -I/usr/local/include/gtsam/3rdparty/Eigen -isystem /usr/include/vtk-7.1 -isystem /usr/include/freetype2 -isystem /usr/include/opencv4 -isystem /usr/include/eigen3 -isystem /usr/include/pcl-1.10 -isystem /usr/include/ni -isystem /usr/include/openni2  -std=c++14 -O3    -o CMakeFiles/imageProjection.dir/src/imageProjection.cpp.o -c /root/catkin_ws/src/LeGO-LOAM/LeGO-LOAM/src/imageProjection.cpp
            cd /root/catkin_ws/build/LeGO-LOAM/LeGO-LOAM && /usr/bin/c++  -DDISABLE_LIBUSB_1_0 -DDISABLE_PCAP -DDISABLE_PNG -DROSCONSOLE_BACKEND_LOG4CXX -DROS_BUILD_SHARED_LIBS=1 -DROS_PACKAGE_NAME=\"lego_loam\" -Dqh_QHpointer -DvtkRenderingContext2D_AUTOINIT="1(vtkRenderingContextOpenGL2)" -DvtkRenderingCore_AUTOINIT="3(vtkInteractionStyle,vtkRenderingFreeType,vtkRenderingOpenGL2)" -I/root/catkin_ws/src/LeGO-LOAM/LeGO-LOAM/include -I/root/catkin_ws/devel/include -I/opt/ros/noetic/include -I/opt/ros/noetic/share/xmlrpcpp/cmake/../../../include/xmlrpcpp -I/usr/local/include/gtsam/3rdparty/Eigen -isystem /usr/include/vtk-7.1 -isystem /usr/include/freetype2 -isystem /usr/include/opencv4 -isystem /usr/include/eigen3 -isystem /usr/include/pcl-1.10 -isystem /usr/include/ni -isystem /usr/include/openni2  -std=c++14 -O3    -o CMakeFiles/featureAssociation.dir/src/featureAssociation.cpp.o -c /root/catkin_ws/src/LeGO-LOAM/LeGO-LOAM/src/featureAssociation.cpp
            In file included from /root/catkin_ws/src/LeGO-LOAM/LeGO-LOAM/include/utility.h:22,
                            from /root/catkin_ws/src/LeGO-LOAM/LeGO-LOAM/src/transformFusion.cpp:33:
            /usr/include/pcl-1.10/pcl/filters/voxel_grid.h: In member function ‘std::vector<int> pcl::VoxelGrid<PointT>::getNeighborCentroidIndices(const PointT&, const MatrixXi&) const’:
            /usr/include/pcl-1.10/pcl/filters/voxel_grid.h:340:21: error: ‘Index’ is not a member of ‘Eigen’
            340 |         for (Eigen::Index ni = 0; ni < relative_coordinates.cols (); ni++)
            ```  
        - this (generated with catkin_make VERBOSE=1) seems to confirm that gtsam Eigen is causing the error.
        - unfortunately, updating to Gtsam 4.0.3 can results in 
            ```
            cc1plus: error: bad value (‘tigerlake’) for ‘-march=’ switch
            ```
            when running 'make install'. This indicates a compiler too old for your hardware/cpu. 
            - related links
                - https://stackoverflow.com/questions/64493692/cc1plus-error-bad-value-tigerlake-for-march-switch-compilation-error 
                - https://stackoverflow.com/questions/1516609/difference-between-cc-gcc-and-g
                - https://stackoverflow.com/questions/45933732/how-to-specify-a-compiler-in-cmake
            - can be resolved using environment variables, if you don't want to change the system default compiler
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
- the static transforms in run.launch
    ```
    <node pkg="tf" type="static_transform_publisher" name="camera_init_to_map"  args="0 0 0 1.570795   0        1.570795 map    camera_init 10" />
    <node pkg="tf" type="static_transform_publisher" name="base_link_to_camera" args="0 0 0 -1.570795 -1.570795 0        camera base_link   10" />
    ```
    - not really sure what these signify. Seems to do things like map the x-axis of one frame to the z-axis of another. Possibly LOAM related.

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

## LeGO-LOAM Sample Bag

- publishes two topics
- /velodyne_points 
    - frame_id: "velodyne"
- /imu/data
    - frame_id: "imu_link"


