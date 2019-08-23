# PLCC
Plane constraint based Lidar-Camera Calibration.  

## 1. Quick Start
We assume you have [setup ROS enviroment](http://wiki.ros.org/ROS/Installation).

### 1.1 Installation

**WARNING: the master branch is in development, please checkout v1.0 branch to get stable and usable code.**

```
git clone https://github.com/rayleizhu/plcc.git
cd plcc
git checkout v1.0
bash install.bash
source devel/setup.bash
```

The installation script is tested on Ubuntu 18.04. If you fail to install some python library, please install it mannually. See `requirements.txt`.

### 1.2 Test with sample data
We provided sample data`data/input/data.bag`, you can run a test by executing the following command
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
The main idea is to use planes to construct correspondence constraints: for each plane we can derive its equation in both camera frame (e.g. using Apriltag detection or Chessboard) and LiDAR frame (fitting point cloud patch), then the normal can be used to constraint roation, and intersection of multiple planes can be used to constraint translation.

![methodology.png](https://github.com/rayleizhu/plcc/blob/master/assets/how_it_works.png "how it works")

## 5. Explanation of files
During initial development, I deliberately partition the tool into several modules, many of them can be run seperately. This is for convenience of debugging and readability. It's easy to understand each module by the file name, here I briefly introduce what each file does:

### 5.1 Core scripts
* `preprocess.py`: it helps you extract image captured by camera, camera-tag transformation, camera info, etc. from rosbag file.  
* `fit_plane_in_pcd.py`: fits plane equation from point cloud patches.  
* `fina_plane_in_cam.py`: get plane equation from cam-tag transformation.  
* `solve_tf.py`: solve Lidar-Camera extrinsic based on plane constraints.  
* `project_pcd_to_img.py`: for reprojection.  
The above scripts can be run seperately (you can run any of them with `-h` option to see its arguments) for calibration, or be imported for further development.

### 5.2 Other scripts
* `run_all.sh`: it connect the whole pipeline with bash script, so that you can run calibration with one command.
* `camera_model.py`: as its name implies, it includes camera model implementation. Currently, pinhole camera and MEI camera are provided.
* `utility.py`: some commonly used functions are put into this file.
* `generate_target.py`: you can generate calibration target (April tag or ChArucoBoard) with it.
* `diff_extr.py`: solve relative transformation. It's useful to validate calibration precision in a 'differential' manner, e.g., let's say you know the extrinsics between left eye and right eye of ZED stereo camera, and you derive both LiDAR-left extrinsic and LiDAR-right extrinsic with our tool, then you can get 'measured' left-right extrinsic with `diff_extr.py`, finally, you may validate the predcision by comparing it with groud truth extrinsic.

## 6. Related projects
[selected_points_publisher](https://github.com/tu-rbo/turbo-ros-pkg): We refer to this repo to write our rviz plugin for point cloud patch selection.  
[camodocal](https://github.com/hengli/camodocal): We use this repo to calibrate camera intrinsics.  
[apriltag_ros](https://github.com/AprilRobotics/apriltag_ros): We use it to get camera-tag transformation.


## 7. TODO
* Refactor ChArucoBoard detection part
* Complete part 4 to introduce how it works
* Refactor the code so that we can run the calibration with one python script and one config file
* Finish the transformation calculator GUI tool
