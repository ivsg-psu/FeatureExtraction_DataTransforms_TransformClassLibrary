function [yaw_offset, angle_array] = fcn_Transform_CalculateYawError(rawdata,process_range)

[GPS_SparkFun_Front_ENU_interp, GPS_SparkFun_LeftRear_ENU_interp,GPS_SparkFun_RightRear_ENU_interp,TimeAligned] = fcn_GPSDataPreprocess(rawdata);


GPS_SparkFun_Front_ENU_selected = GPS_SparkFun_Front_ENU_interp(process_range,:);
GPS_SparkFun_LeftRear_ENU_selected = GPS_SparkFun_LeftRear_ENU_interp(process_range,:);
GPS_SparkFun_RightRear_ENU_selected = GPS_SparkFun_RightRear_ENU_interp(process_range,:);
traj_range = 1:(length(process_range)+1);
V_front_traj = diff(GPS_SparkFun_Front_ENU_interp(traj_range,:),1,1);
V_left_traj = diff(GPS_SparkFun_LeftRear_ENU_interp(traj_range,:),1,1);
V_right_traj = diff(GPS_SparkFun_RightRear_ENU_interp(traj_range,:),1,1);

V_right_left = GPS_SparkFun_LeftRear_ENU_selected - GPS_SparkFun_RightRear_ENU_selected;

[angle_right,angle_right_deg] = fcn_Transform_CalculateAngleBetweenVectors(V_right_left,V_right_traj);
[angle_left,angle_left_deg] = fcn_Transform_CalculateAngleBetweenVectors(V_right_left,V_left_traj);

angle_array = (angle_right+angle_left)/2 - pi/2;
yaw_offset = mean(angle_array,1);