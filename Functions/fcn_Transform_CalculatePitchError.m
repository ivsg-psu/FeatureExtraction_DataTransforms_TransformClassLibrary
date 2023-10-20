function [pitch_offset, pitch_array,V_right_virtual] = fcn_Transform_CalculatePitchError(rawdata,process_range,yaw_offset)

[GPS_SparkFun_Front_ENU_interp, GPS_SparkFun_LeftRear_ENU_interp,GPS_SparkFun_RightRear_ENU_interp,TimeAligned] = fcn_GPSDataPreprocess(rawdata);


GPS_SparkFun_Front_ENU_selected = GPS_SparkFun_Front_ENU_interp(process_range,:);
GPS_SparkFun_LeftRear_ENU_selected = GPS_SparkFun_LeftRear_ENU_interp(process_range,:);
GPS_SparkFun_RightRear_ENU_selected = GPS_SparkFun_RightRear_ENU_interp(process_range,:);
traj_range = 1:(length(process_range)+1);
V_front_traj = diff(GPS_SparkFun_Front_ENU_interp(traj_range,:),1,1);
V_front_traj_mag = vecnorm(V_front_traj,2,2);
V_front_traj_unit = V_front_traj./V_front_traj_mag;



V_right_traj = diff(GPS_SparkFun_RightRear_ENU_interp(traj_range,:),1,1);
V_right_traj_mag = vecnorm(V_right_traj,2,2);
V_right_traj_unit = V_right_traj./V_right_traj_mag;


V_right_front = GPS_SparkFun_Front_ENU_selected - GPS_SparkFun_RightRear_ENU_selected;
V_right_left = GPS_SparkFun_LeftRear_ENU_selected - GPS_SparkFun_RightRear_ENU_selected;
V_right_left_mag = vecnorm(V_right_left,2,2);
V_right_left_unit = V_right_left./V_right_left_mag;
V_projection = fcn_Transform_VectorProjection(V_right_front, V_right_left_unit); % The projection is along V_y_unit, Nx3
V_right_virtual = V_right_front - V_projection;
% V_right_virtual(:,3) = V_right_virtual(:,3) + 1;
V_right_virtual_mag = vecnorm(V_right_virtual,2,2);
V_right_virtual_unit = V_right_virtual./V_right_virtual_mag;
% 
V_sign = cross(V_right_traj_unit,V_right_virtual_unit);
pitch_direction = sign(V_sign(:,2));
[angle_right,angle_right_deg] = fcn_Transform_CalculateAngleBetweenVectors(V_right_virtual,V_right_traj);
[angle_front,angle_front_deg] = fcn_Transform_CalculateAngleBetweenVectors(V_right_virtual,V_front_traj);
angle_array = (angle_right + angle_front)/2;

L = V_right_virtual_mag.*sin(angle_array);
d = V_right_virtual_mag.*sin(yaw_offset);
h = sqrt(L.^2 - d.^2);
pitch_mag_array = asin(h./V_right_virtual_mag);
pitch_array = pitch_direction.*pitch_mag_array;
pitch_offset = mean(angle_array,1);