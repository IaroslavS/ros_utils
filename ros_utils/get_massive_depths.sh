#!/usr/bin/env bash
declare id
declare name
declare url
declare version

eval "$(conda shell.bash hook)"
conda activate python27
python --version

path_to_KITTI_dataset=/media/cds-s/data/Datasets/KITTI_Odometry/dataset/sequences
rate=3.0

cd /home/cds-s/workspace/ros_utils_ws/src/ros_utils/ros_utils/launch

source /home/cds-s/workspace/ros_utils_ws/devel/setup.bash

#: << comment
for dataset_path in /media/cds-s/data/Datasets/KITTI_Odometry/dataset/sequences/*; do
   export sequencenumber=${dataset_path##*/}
   case "$sequencenumber" in
     00) # |01|02|03|04|05|06|07|"08")  |$done_rosbag16) # |$done_rosbag2|$done_rosbag3|$done_rosbag4|$done_rosbag5) # |
     continue ;;
   esac
   echo ""
   echo "Processing sequence $sequencenumber"
   echo ""
   roslaunch get_esgm_depth_maps.launch output_path:=$dataset_path/RGBD/depth rate:=$rate fname_left:=$dataset_path/image_0/* fname_right:=$dataset_path/image_1/* sequence_number:=$sequencenumber
done

#comment

