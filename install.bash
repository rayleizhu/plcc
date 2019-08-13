# /bin/bash

# install rviz plugin
catkin_make install

# install python dependencies
pip2 install -r requirements.txt

# create directory for data
mkdir -p data/output/pcd_patches data/input