function [yaw_offset_array, pitch_offset_array,V_virtual_frame] = fcn_Transform_calculateYawAndPitchOffset(rawdata,process_range)
% Note: This function is very abstract, containing a virtual frame
% that do not exist, will add figures to explain the function
% Vehicle Coordinate Frame： X-Y-Z
% V_x_vehicle: X-axis of Vehicle Coordinate Frame
% V_y_vehicle: Y-axis of Vehicle Coordinate Frame
% V_z_vehicle: Z-axis of Vehicle Coordinate Frame

% Virtual Coordinate Frame： X'-Y'-Z'
% V_x_virtual = V_x_vehicle;
% V_y_virtual: Y-axis of a virtual frame used to calculate offset
% V_z_virtual: Z-axis of a virtual frame used to calculate offset
%% Step 1 - Preprocess the GPS data, all unlocked data will be removed and time will be synchronized (Temp version, will update later)
[GPS_SparkFun_Front_ENU_interp, GPS_SparkFun_LeftRear_ENU_interp,GPS_SparkFun_RightRear_ENU_interp,TimeAligned] = fcn_GPSDataPreprocess(rawdata);

%% Step 2 - Select the process range
GPS_SparkFun_Front_ENU_selected = GPS_SparkFun_Front_ENU_interp(process_range,:);
GPS_SparkFun_LeftRear_ENU_selected = GPS_SparkFun_LeftRear_ENU_interp(process_range,:);
GPS_SparkFun_RightRear_ENU_selected = GPS_SparkFun_RightRear_ENU_interp(process_range,:);
traj_range = 1:(length(process_range)+1);

%% Step 1: Aveage three GPS antennas moving trajectories, use the average vector as the X axis of the vehicle frame
V_front_traj = fcn_Calibration_CalculateGPSTrajectory(GPS_SparkFun_Front_ENU_interp,traj_range);
V_left_traj = fcn_Calibration_CalculateGPSTrajectory(GPS_SparkFun_LeftRear_ENU_interp,traj_range);
V_right_traj = fcn_Calibration_CalculateGPSTrajectory(GPS_SparkFun_RightRear_ENU_interp,traj_range);
V_x_vehicle = mean([V_front_traj;V_left_traj;V_right_traj]);
V_x_vehicle_unit = normalizeVector(V_x_vehicle); % X-axis of Vehicle Coordinate Frame
%% Step 2: Find the yaw angle offset
V_right_left = GPS_SparkFun_LeftRear_ENU_selected - GPS_SparkFun_RightRear_ENU_selected;
% Calculate and normalize V_right_left vector, ideally, this vector is the Y axis of the
% vehicle frame, however, instllation error results in roll and yaw offsets
V_right_left_unit = normalizeVector(V_right_left);
V_x_vehicle_array = repmat(V_x_vehicle_unit,size(V_right_left_unit,1),1);

% Project V_right_left on the V_x_vehicle_unit to decompose the vector
% into two orthogonal vectoris, where V_right_left_x_proj is the vector
% along V_x_vehicle, while V_y_virtual is a vector perpendicular to
% V_x_vehicle, which indicates that there is only roll offset between
% V_y_virtual and V_y_vehicle, and the roll offset will be calculated later
V_right_left_x_proj = fcn_Transform_VectorProjection(V_right_left_unit,V_x_vehicle_array);
V_y_virtual = V_right_left_unit - V_right_left_x_proj;
% Calculate the yaw offsets by calculating the angle between V_right_left
% and V_y_virtual, from V_right_left to V_y_virtual
V_yaw_sign = cross(V_right_left_unit,V_y_virtual);
yaw_direction = sign(V_yaw_sign(:,3));
yaw_offset_mag_array = fcn_Transform_CalculateAngleBetweenVectors(V_right_left_unit,V_y_virtual);
yaw_offset_array = yaw_direction.*yaw_offset_mag_array;
%%
% Use V_x_vehicle and V_y_vitrual to find another V_z_virtual, similar to
% V_y_vitrual, V_z_virtual only has roll offset, which will be calculated later
V_z_virtual = cross(V_x_vehicle_array,V_y_virtual); % A unit vector

% Calculate and normalize V_right_front vector, ideally, this vector is the 
% x axis of the vehicle frame, however, front GPS antenna was not installed 
% aligned with the rear right GPS antenna
V_right_front = GPS_SparkFun_Front_ENU_selected - GPS_SparkFun_RightRear_ENU_selected;
V_right_front_unit = normalizeVector(V_right_front);
% Project V_right_front_unit on the V_y_virtual to decompose the vector
% into two orthogonal vectoris, where V_right_front_y_virtual_proj is the 
% vector along V_y_virtual while V_right_front_shift is a vector in the
% X'Z' plane
V_right_front_y_virtual_proj = fcn_Transform_VectorProjection(V_right_front_unit, V_y_virtual);
V_right_front_shift = V_right_front_unit - V_right_front_y_virtual_proj;
% Calculate the pitch offsets by calculating the angle between V_right_front_shift
% and V_x_vehicle, from V_right_front_shift to V_x_vehicle
V_pitch_sign = cross(V_right_front_shift,V_x_vehicle_array);
pitch_direction = sign(V_pitch_sign(:,2));
[pitch_offset_mag_array] = fcn_Transform_CalculateAngleBetweenVectors(V_right_front_shift,V_x_vehicle_array);
pitch_offset_array = pitch_direction.*pitch_offset_mag_array;

V_virtual_frame = struct;
V_virtual_frame.X = V_x_vehicle_array;
V_virtual_frame.Y = V_y_virtual;
V_virtual_frame.Z = V_z_virtual;
end
%% Normalize vector to unit vector
function V_unit = normalizeVector(V)
    V_mag = vecnorm(V,2,2);
    V_unit = V./V_mag;
end
