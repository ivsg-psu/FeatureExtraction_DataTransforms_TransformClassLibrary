function [yaw_offset, pitch_offset, yaw_array, pitch_array] = fcn_Transform_GPSInstallationCalibration(rawdata,process_range)


[GPS_SparkFun_Front_ENU_interp, GPS_SparkFun_LeftRear_ENU_interp,GPS_SparkFun_RightRear_ENU_interp,TimeAligned] = fcn_GPSDataPreprocess(rawdata);


GPS_SparkFun_Front_ENU_selected = GPS_SparkFun_Front_ENU_interp(process_range,:);
GPS_SparkFun_LeftRear_ENU_selected = GPS_SparkFun_LeftRear_ENU_interp(process_range,:);
GPS_SparkFun_RightRear_ENU_selected = GPS_SparkFun_RightRear_ENU_interp(process_range,:);
traj_range = 1:(length(process_range)+1);

V_front_traj = diff(GPS_SparkFun_Front_ENU_interp(traj_range,:),1,1);
V_front_traj_mag = vecnorm(V_front_traj,2,2);
V_front_traj_unit = V_front_traj./V_front_traj_mag;

V_left_traj = diff(GPS_SparkFun_LeftRear_ENU_interp(traj_range,:),1,1);

V_right_traj = diff(GPS_SparkFun_RightRear_ENU_interp(traj_range,:),1,1);
V_right_traj_mag = vecnorm(V_right_traj,2,2);
V_right_traj_unit = V_right_traj./V_right_traj_mag;

V_right_left = GPS_SparkFun_LeftRear_ENU_selected - GPS_SparkFun_RightRear_ENU_selected;

[yaw_right,yaw_right_deg] = fcn_Transform_CalculateAngleBetweenVectors(V_right_left,V_right_traj);
[yaw_left,yaw_left_deg] = fcn_Transform_CalculateAngleBetweenVectors(V_right_left,V_left_traj);

yaw_array = (yaw_right+yaw_left)/2 - pi/2;
yaw_offset = mean(yaw_array,1);



V_right_front = GPS_SparkFun_Front_ENU_selected - GPS_SparkFun_RightRear_ENU_selected;
V_right_left = GPS_SparkFun_LeftRear_ENU_selected - GPS_SparkFun_RightRear_ENU_selected;
V_right_left_mag = vecnorm(V_right_left,2,2);
V_right_left_unit = V_right_left./V_right_left_mag;
V_projection = fcn_Transform_VectorProjection(V_right_front, V_right_left_unit); % The projection is along V_y_unit, Nx3
V_right_virtual = V_right_front - V_projection;
V_right_virtual_mag = vecnorm(V_right_virtual,2,2);
V_right_virtual_unit = V_right_virtual./V_right_virtual_mag;
% 
V_sign = V_right_virtual_unit - V_right_traj_unit;
pitch_direction = -sign(V_sign(:,3));
[angle_right,angle_right_deg] = fcn_Transform_CalculateAngleBetweenVectors(V_right_virtual,V_right_traj);
[angle_front,angle_front_deg] = fcn_Transform_CalculateAngleBetweenVectors(V_right_virtual,V_front_traj);
angle_array = (angle_right+angle_front)/2;

d = V_right_virtual_mag.*cos(angle_array);
L = d./cos(abs(yaw_array));


% h = sqrt(L.^2 - d.^2);
pitch_mag_array = acos(L./V_right_virtual_mag);
pitch_array = pitch_direction.*pitch_mag_array;
pitch_array_valid = pitch_array(L<=V_right_virtual_mag);
pitch_offset = mean(pitch_array_valid,1);