#! /bin/sh
# run_calibration.sh
# Copyright (C) 2018 ckddls1321 <ckddls1321@SALinuxckddls>
# Distributed under terms of the MIT license.
if [ "$#" -eq 3 ] || [ "$#" -eq 4 ]; then
echo "#-----------------------------------------------------\n"
echo "Start Calibration\n"
echo "#-----------------------------------------------------\n"
#---------------------------------------------------------
# PARAMETER NAME
#---------------------------------------------------------
export IMUTOPIC=$1
export ROSLAUNCH=$2
export CAM0TOPIC=$3   #/cam0/image_raw
if [ "$#" -ne 3 ];
then
export CAM1TOPIC=
elif [ "$#" -ne 4 ];
then
export CAM1TOPIC=$4   #/cam1/image_raw
fi
#---------------------------------------------------------
export CAMMODEL=pinhole-radtan #pinhole-equi omni-radtan
export CALBAG=static.bag
export STATICBAG=static.bag
export IMUBAG=imu_cal.bag
export DYNAMICBAG=dynamic.bag
export CALTARGET=april_6x6_43x43cm.yaml
export CAMCHAIN=camchain-static.yaml
export IMUCHAIN=imu_cal.yaml
export CAMIMUCHAIN=camchain-imucam-dynamic.yaml
#---------------------------------------------------------
# Kalibr Camera Calibration : static.bag
#---------------------------------------------------------
# 0) Enter DIR
cd static
# 1) Camera Calibration
kalibr_calibrate_cameras --models $CAMMODEL --topics $CAM0TOPIC $CAM1TOPIC --dont-show-report --bag $CALBAG --target $CALTARGET --bag-from-to 10 60
cp $CAMCHAIN  ../dynamic/$CAMCHAIN
# 2) Exit DIR
cd ..
#---------------------------------------------------------
# IMU Intrinsic Calibration : imu.bag with 1HR static data
#---------------------------------------------------------
cd imu
# 1) IMU intrinsic Calibration : Allan Variance
# Put launch file below catkin_ws/src/allan_variance/launch
#roslaunch allan_variance $ROSLAUNCH
# 2) Make Results to kalibr format : read txt to make imu_cal.yaml
cp $IMUCHAIN ../dynamic/$IMUCHAIN
# 3) Validate IMU intrinsic value : how?

cd ..
#---------------------------------------------------------
# Kalibr Camera_IMU Calibration : dynamic.bag 70s
#---------------------------------------------------------
cd dynamic
# 1) Camera IMU calibration using camchain & imu : using mid 50s
# Important Flag list
# --imu-models scale-misalignment   :
# --reprojection-sigma [param]      :
# --verbose							:
# --dont-show-report                :
# --show-extraction					:
# --extraction-stepping				:
# --time-calibration				:
# --perform-synchronization			:
kalibr_calibrate_imu_camera --target $CALTARGET --cam camchain-static.yaml --imu imu_cal.yaml --perform-synchronization --time-calibration --dont-show-report --bag $DYNAMICBAG --bag-from-to 10 65
# 2) Change Config to other format : okvis, rovio, msckf, ORB, SVO
kalibr_okvis_config --cam $CAMIMUCHAIN --out ${BASETOPIC}_cal.yaml
kalibr_rovio_config --cam $CAMIMUCHAIN
# 3) Validate Transform Matrix
cd ..

else
 	echo "Usage of Scripts test.sh "
 	echo "Parameter Number 2[Mono] 3[Stereo]"
	echo "\$1 : IMUTOPIC : /imu0, /dvs/imu "
	echo "\$2 : ROS Launch File : davis.launch, cam2pc.launch "
	echo "\$2 : CAM0TOPIC : /cam0/image_raw, /dvs/image_raw"
	echo "\$3 : CAM1TOPIC : /cam1/image_raw"
fi


