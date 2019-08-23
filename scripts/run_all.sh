#! /bin/bash

# Author: Zhu Lei
# Email: leifzhu@foxmail.com

###############config region########################

#DATA_ROOT=/home/rayleizhu/ros-project/plcc/data
DATA_ROOT=/home/rayleizhu/Data/LidarCamCalib/data0823/first_right
NTAG=3
IMG_TOPIC=/zed/zed_node/right/image_rect_color
CAM_TOPIC=/zed/zed_node/right/camera_info
PCD_FRAME=velodyne
PCD_READY=false

COS_TH=0.95


####################################################
# Path of inputs
PCD_DIR=$DATA_ROOT/output/pcd_patches
BAG=$DATA_ROOT/input/data.bag

# Path of outputs
IMG_ORIG=$DATA_ROOT/output/orig.png
CAM_INFO=$DATA_ROOT/output/camera_info.yaml
CAM_TAG_TF=$DATA_ROOT/output/tf_tag_to_cam.csv
PLANES_PCD=$DATA_ROOT/output/planes_pcd.csv
PLANES_CAM=$DATA_ROOT/output/planes_cam.csv
ITSC_SRC=$DATA_ROOT/output/intersection_src.csv
ITSC_TGT=$DATA_ROOT/output/intersection_tgt.csv
CALIB_RESULT=$DATA_ROOT/output/calib_result.csv
IMG_RPJ=$DATA_ROOT/output/reprojection.png

####################################

# cd to plcc root folder, start rviz for pcd patch selection
if ! $PCD_READY; then
    cd ..
    source devel/setup.bash
    rosbag play -q $BAG &
    rviz -f $PCD_FRAME

    PLCC_ROOT=$(pwd)
    STD_DATA_ROOT=$PLCC_ROOT/data
    GIVEN_DATA_ROOT=$(cd "$DATA_ROOT";pwd)

    if [ "$STD_DATA_ROOT" != "$GIVEN_DATA_ROOT" ]; then
        mkdir -p $GIVEN_DATA_ROOT/output/pcd_patches/
        mv $STD_DATA_ROOT/output/pcd_patches/* $GIVEN_DATA_ROOT/output/pcd_patches/
    fi
fi

# return scripts folder
cd scripts

python preprocess.py $NTAG -i $BAG -t $IMG_TOPIC -m $IMG_ORIG -f $CAM_TAG_TF \
                    -c $CAM_TOPIC -y $CAM_INFO
python fit_plane_in_pcd.py -i $PCD_DIR -o $PLANES_PCD -c $COS_TH
python find_plane_in_cam.py -i $CAM_TAG_TF -o $PLANES_CAM -c $COS_TH
python solve_tf.py -s $PLANES_PCD -t $PLANES_CAM -o $CALIB_RESULT -c $COS_TH \
                    --itsc-src $ITSC_SRC --itsc-tgt $ITSC_TGT
python project_pcd_to_img.py -p $PCD_DIR \
                             --itsc-pcd $ITSC_SRC \
                             --itsc-cam $ITSC_TGT \
                             -m $IMG_ORIG \
                             -t $CALIB_RESULT \
                             -c $CAM_INFO \
                             -o $IMG_RPJ




