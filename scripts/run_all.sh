#! /bin/bash

DATA_ROOT=$1
NTAG=$2
COS_TH=0.95

# Path of inputs
PCD_DIR=$DATA_ROOT/output/pcd_patches
CAM_TAG_TF=$DATA_ROOT/output/tf_tag_to_cam.csv
###################
CAM_INTR=$DATA_ROOT/output/intr.yaml
BAG=$DATA_ROOT/input/data.bag
IMG_TOPIC=/zed/zed_node/left/image_rect_color

# Path of outputs
PLANES_PCD=$DATA_ROOT/output/planes_pcd.csv
PLANES_CAM=$DATA_ROOT/output/planes_cam.csv
RESULT=$DATA_ROOT/output/calib_result.csv
IMG_ORIG=$DATA_ROOT/output/orig.png
IMG_RPJ=$DATA_ROOT/output/reprojection.png

python preprocess.py $NTAG -i $BAG -t $IMG_TOPIC -m $IMG_ORIG -f $CAM_TAG_TF
python fit_plane_in_pcd.py -i $PCD_DIR -o $PLANES_PCD -c $COS_TH
python find_plane_in_cam.py -i $CAM_TAG_TF -o $PLANES_CAM -c $COS_TH
python solve_tf.py -s $PLANES_PCD -t $PLANES_CAM -o $RESULT -c $COS_TH




