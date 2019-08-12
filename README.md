# PLCC
Plane constraint based Lidar-Camera Calibration

## Quick Start
We assume you have setup ROS enviroment.

### install rviz plugin

```
git clone https://github.com/rayleizhu/plcc.git
cd plcc
catkin_make install
```

### install python dependencies

```
pip2 install -r requirements.txt
``` 

### use rviz plugin to select point cloud patches

```
source devel/setup.bash
mkdir -p data/output/pcd_patches data/input
```

Then you can start rviz to select point cloud patches. Note that,  
1. if you want to reset patch count which is set for csv file name generation, you need to restart rviz.
2. if you use conda environment, there may be conflict between conda envs and ros envs.

### use python scripts to solve transformation
```
cd scripts
python fit_plane_in_pcd.py

... ...
```
