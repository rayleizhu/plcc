# PLCC
Plane constraint based Lidar-Camera Calibration

## 1. Quick Start
We assume you have [setup ROS enviroment](http://wiki.ros.org/ROS/Installation).

### 1.1 One-click installation

```
git clone https://github.com/rayleizhu/plcc.git
cd plcc
bash install.bash
source devel/setup.bash
```
### 1.2 Test with sample data
We provided sample data`data/input/data.bag`, you can run a test by runing the following command
```
cd scripts
bash run_all.bash
```
When the script terminated, you can get results in the directory `data/output/`. Two import files are:  
* calib_result.csv: containing calibration result (R, t) which is transformation from lidar to camera
* reprojection.png: the reprojection result using calibrated realative transformation.

Data directory structure after calibration:
![data_dir_tree.png](https://github.com/rayleizhu/plcc/blob/master/assets/data_dir_tree.png "Directory structure")


Example of reprojected result:
![reprojection.png](https://github.com/rayleizhu/plcc/blob/master/data/output/reprojection.png "reprojection result sample")


## 2. Run calibration with your own data

### 2.1 Procedures 
1. Replace the sample data `data/input/data.bag` with your own recorded bag file. Refer to part 2.2 for requirement of the ROS bag file.
2. Modify configurations in scripts/run_all.sh
3. Run the following commands
```
cd scripts
bash run_all.bash
```

### 2.2 Requirement
You can use `rosbag info data/input/data.bag` to figure out how the input bag should be like: 

![data_bag_info.png](https://github.com/rayleizhu/plcc/blob/master/assets/data_bag_info.png "Data bag info")

Topics:
* /tf: should containing transformations from each tag to camera. You can use [apriltag_ros](https://github.com/AprilRobotics/apriltag_ros) to achieve this goal.  
* /velodyne_points: the topic containing point cloud. The name of this topic is depend on the sensor you are using. Importantly, you need to change `PCD_FRAME` in `run_all.bash` corrspondingly as the frame point cloud resides in.
* /zed/zed_node/left/image_rect_color: the topic containing images camera shot. Corresponding variable in `run_all.bash` is `IMG_TOPIC`.
* /zed/zed_node/left/camera_info: the topic containing images camera information (intrinsics, etc.). Corresponding variable in `run_all.bash` is `CAM_TOPIC`.

## 3. How to use our rviz plugin for point cloud patch selection
hold `ALT`: moving mode  
hold `SHIFT`: incremental selection    
hold `CTRL`: inverse selection  
press `F`: focus on selected points  
press `P`: pushlish selected patch  

## 4. How it works

## 5. Related projects
[selected_points_publisher](https://github.com/tu-rbo/turbo-ros-pkg): We refer to this repo to write our rviz plugin for point cloud patch selection.  
[camodocal](https://github.com/hengli/camodocal): We use this repo to calibrate camera intrinsics.  
[apriltag_ros](https://github.com/AprilRobotics/apriltag_ros): We use it to get camera-tag transformation.

## 6. TODO
* Add [CharucoBoard detection](https://docs.opencv.org/3.2.0/d0/d3c/classcv_1_1aruco_1_1CharucoBoard.html)  
* Complete part 4 to introduce how it works
* Refactor the code so that we can run the calibration with one python script and one config file
