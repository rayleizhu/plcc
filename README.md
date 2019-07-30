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
It's recommended to use conda virtual environment:

```
conda create -n plcc python=3.6
conda install --yes --file requirements.txt 
```

Or if you want to use pip:
```
pip3 install -r requirements.txt
``` 

### use rviz plugin to select point cloud patches

````
source devel/setup.bash
mkdir -p data/output/pcd_patches data/input
```

Then you can start rviz to select point cloud patches. Note that, if you want reset patch count which is set for csv file name generation, you need to restart rviz. 

### use python scripts to solve transformation
