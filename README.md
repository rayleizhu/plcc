# PLCC
Plane constraint based Lidar-Camera Calibration

## Quick Start
We assume you have setup ROS enviroment.

### One-click installation

```
git clone https://github.com/rayleizhu/plcc.git
cd plcc
bash install.bash
source devel/setup.bash
```

### use python scripts to solve transformation
1. Put your recored bag file into data/input/data.bag 
2. Modify configurations in scripts/run_all.sh
3. Run run_all.sh:
```
cd scripts
bash run_all.sh
```

Note that,  
1. if you want to reset patch count which is set for csv file name generation, you need to restart rviz.
2. if you use conda environment, there may be conflict between conda envs and ros envs