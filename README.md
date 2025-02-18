# VAVM
A Variable Angular Velocity Model (VAVM) and An Event-Based Fast Rotation Dataset.

#
Rotation estimation algorithms implemented in the VAVM model, including the EF-based approach (VAVM-A) and TS-based method (VAVM-B), along with our proposed optimized cost function.


# The Event-based Fast Rotation Dataset (EFRD)
The dataset is available at [EFRD](https://drive.google.com/drive/folders/1tOlC-aOPL0YmYcQKetyEFWWRYONG8kF0).

## Data Format
The EFRD is recorded in the data format of rosbag. Within the rosbag of each sequence, the following messages are mainly included:

- /dvs/events (dvs_msgs/EventArray)  
  The events captured by DAVIS346. 
- /dvs/image_raw (sensor_msgs/Image)  
  The frames captured by DAVIS346.
- /dvs/camera_info (sensor_msgs/CameraInfo)  
  The intrinsics of DAVIS346.
- /imu/data (sensor_msgs/Imu)  
  The raw data produced by LPMS-IG1 IMU.
- /imu/gt (geometry_msgs/PoseStamped)  
  The ground truth of the position and orientation of DAVIS346. Generated from data in "/imu/data". 

## Calibration
The intrinsics of DAVIS346 is calibrated by VINS-mono and is reported in _cam_calib.yaml_ (same with data in "/dvs/camera_info"). The extrinsics between DAVIS346 and IMU is calibrated by Kalibr and is reported in _imucam_calib.yaml_.

# Reference

Some other good rotation estimation resources that we benefit from:

[CMAX-GAE]

[STAE]

If you utilize any portion of this codebase, please kindly cite the abovementioned seminal works.
