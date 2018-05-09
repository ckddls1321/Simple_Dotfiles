#!/bin/sh
#mkdir EUROC
#mkdir KITTI
#mkdir PENNCOSYVIO
#mkdir MVSEC
#mkdir TUM

#cd EUROC

#wget http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_03_medium/MH_03_medium.zip &
#wget http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room1/V1_02_medium/V1_02_medium.zip &
#wget http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room2/V2_02_medium/V2_02_medium.zip &

#cd ..

#cd KITTI
#wget http://kitti.is.tue.mpg.de/kitti/data_odometry_color.zip &
#wget http://kitti.is.tue.mpg.de/kitti/data_odometry_calib.zip &

#cd ..

#cd PENNCOSYVIO
#wget http://visiondata.cis.upenn.edu/penncosyvio/tarfiles/visensor.tar &
#wget http://visiondata.cis.upenn.edu/penncosyvio/tarfiles/intrinsic_calib.tar &
#wget http://visiondata.cis.upenn.edu/penncosyvio/tarfiles/extrinsic_calib.tar &
#
#cd ..

cd MVSEC
wget http://visiondata.cis.upenn.edu/mvsec/outdoor_day/outdoor_day1_data.bag &
wget http://visiondata.cis.upenn.edu/mvsec/indoor_flying/indoor_flying1_data.bag &
wget http://visiondata.cis.upenn.edu/mvsec/indoor_flying/indoor_flying_calib.zip &
wget http://visiondata.cis.upenn.edu/mvsec/outdoor_day/outdoor_day_calib.zip &
cd ..

#cd TUM
#wget https://vision.in.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_desk.tgz &
#wget https://vision.in.tum.de/rgbd/dataset/freiburg2/rgbd_dataset_freiburg2_desk.tgz &
#wget https://vision.in.tum.de/rgbd/dataset/freiburg2/rgbd_dataset_freiburg2_large_with_loop.tgz &
#cd ..

