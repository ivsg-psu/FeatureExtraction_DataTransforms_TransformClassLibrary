function [M_rotation_GPS_to_Vehicle,M_rotation_GPS_to_Vehicle_opposite,M_calibration_GPS_to_Vehicle] = fcn_Transform_createCalibrateMatrix(rawdata_1,rawdata_2,ref_basestation)

% fcn_Transform_createCalibrateMatrix takes two rawdata structs as inputs 
% to calculates the yaw and pitch offsets in rad between the vehicle frame
% and virtual GPS frame
%
% FORMAT:
%
%      [yaw_offset_array, pitch_offset_array,V_virtual_frame] = fcn_Transform_calculateYawAndPitchOffset(GPS_data_struct,process_range)
%
% INPUTS:
%
%      GPS_data_struct: a structure array containing Front, Rear Left, and
%      Rear Right GPS data
%
%      process_range: a array containing the indexs range to process, [12:
%      33] ,[24:67], [1,2,3,4,5,6,7,8,9,10] e.g.
%
%      (OPTIONAL INPUTS)
%
%      fig_num: a figure number to plot results. If set to -1, skips any
%      input checking or debugging, no figures will be generated, and sets
%      up code to maximize speed.
%
% OUTPUTS:
%
%      yaw_offset_array: an array containing the yaw offsets in
%      rad between the vehicle frame and the virtual GPS frame
%
%      pitch_offset_array: an array containing the the pitch offsets in
%      rad between the vehicle frame and the virtual GPS frame
%
%      V_virtual_frame: an array containing the vectors composed the
%      virtual GPS frame
%
% DEPENDENCIES:
%
%      fcn_DebugTools_checkInputsToFunctions
%      fcn_geometry_plotCircle
%      fcn_Calibration_CalculateGPSTrajectory
%      fcn_GPSDataPreprocess
%      fcn_Transform_VectorProjection
%      fcn_Transform_CalculateAngleBetweenVectors
%
% EXAMPLES:
%      
%      % BASIC example
%
%      [yaw_offset_array, pitch_offset_array,V_virtual_frame] = fcn_Transform_calculateYawAndPitchOffset(GPS_data_struct,process_range)
% 
% See the script: script_test_fcn_Transform_calculateYawAndPitchOffset
% for a full test suite.
%
% This function was written on 2023_10_20 by X.Cao
% Questions or comments? xfc5113@psu.edu

% Revision history:
% 2023_10_20 - wrote the code
% 2024_02_22 - added more comments, particularly to explain inputs more
% clearly
% 2024_10_07 --

% Note: This function is very abstract, containing a virtual frame
% that do not exist, will add figures to explain the function
% Vehicle Coordinate Frame： X-Y-Z
% V_x_vehicle: X-axis of Vehicle Coordinate Frame
% V_y_vehicle: Y-axis of Vehicle Coordinate Frame
% V_z_vehicle: Z-axis of Vehicle Coordinate Frame

% Virtual GPS Coordinate Frame： X'-Y'-Z'
% V_x_virtual = V_x_vehicle;
% V_y_virtual: Y-axis of a virtual frame used to calculate offset
% V_z_virtual: Z-axis of a virtual frame used to calculate offset

[Basis_GPS_1,Basis_Vehicle_1] = fcn_GPS_Calibration_constructOrthonormalBasis(rawdata_1,ref_basestation);
[Basis_GPS_2,Basis_Vehicle_2] = fcn_GPS_Calibration_constructOrthonormalBasis(rawdata_2,ref_basestation);

%% With two datasets from two opposite directions, vehicle basis can be computed
% Grab the Z-axis unit of the rear right GPS coordinate system
g_z_unit_gps_1 = Basis_GPS_1.Z;
g_z_unit_gps_2 = Basis_GPS_2.Z;
% Use the aveage to find the normal vector of the current road surface
g_z_unit_gps_1_ave = mean(g_z_unit_gps_1,1);
g_z_unit_gps_2_ave = mean(g_z_unit_gps_2,1);
v_z_unit_vehicle_ave = (g_z_unit_gps_1_ave+g_z_unit_gps_2_ave)/2;

% Extract the X-axis unit vector of the vehicle coordinate system
v_x_unit_vehicle_1 = Basis_Vehicle_1.X;
v_x_unit_vehicle_2 = Basis_Vehicle_2.X;
% Calcualte the Y-axis unit vector by applying cross product of the two
% unit vectors
v_y_unit_vehicle_1 = cross(v_z_unit_vehicle_ave,v_x_unit_vehicle_1);
v_y_unit_vehicle_2 = cross(v_z_unit_vehicle_ave,v_x_unit_vehicle_2);
% Calcualte the Z-axis unit vector by applying cross product of the two
% unit vectors
v_z_unit_vehicle_1 = cross(v_x_unit_vehicle_1,v_y_unit_vehicle_1);
v_z_unit_vehicle_2 = cross(v_x_unit_vehicle_2,v_y_unit_vehicle_2);

% Grab the rotation matrix from GPS to ENU coordiante system
M_rotation_GPS_to_ENU = Basis_GPS_1.R;
M_rotation_GPS_to_ENU_opposite = Basis_GPS_2.R;
% Construct the rotation matrix from vehicle to ENU coordinate system with
% unit vectors
M_rotation_Vehicle_to_ENU = [v_x_unit_vehicle_1.', v_y_unit_vehicle_1.', v_z_unit_vehicle_1.'];
M_rotation_Vehicle_to_ENU_opposite = [v_x_unit_vehicle_2.', v_y_unit_vehicle_2.', v_z_unit_vehicle_2.'];
% Calculate the rotation matrix from GPS to vehicle coordinate system and
% then calculate the roll, pitch and yaw angle in rad
M_rotation_GPS_to_Vehicle = (M_rotation_Vehicle_to_ENU\M_rotation_GPS_to_ENU);
[roll, pitch, yaw] = fcn_GPS_Calibration_calculateRollPitchYaw(M_rotation_GPS_to_Vehicle);
M_rotation_GPS_to_Vehicle_opposite = M_rotation_Vehicle_to_ENU_opposite\M_rotation_GPS_to_ENU_opposite;
[roll_opposite, pitch_opposite, yaw_opposite] = fcn_GPS_Calibration_calculateRollPitchYaw(M_rotation_GPS_to_Vehicle_opposite);
roll_ave = (roll+roll_opposite)/2;
pitch_ave = (pitch + pitch_opposite)/2;
yaw_ave = (yaw + yaw_opposite)/2;
% Create the z-rotate matrix
rotation_matrix_z = makehgtform('zrotate',yaw_ave);
% Create the y-rotate matrix
rotation_matrix_y = makehgtform('yrotate',pitch_ave);
% Create the x-rotate matrix
rotation_matrix_x = makehgtform('xrotate',roll_ave);
M_calibration_GPS_to_Vehicle = rotation_matrix_z*rotation_matrix_y*rotation_matrix_x;
end
% roll_offset_Vehicle = fcn_Transform_calculateRollOffset(v_y_unit_vehicle_1,v_y_unit_vehicle_2);
%% Functions follow
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   ______                _   _
%  |  ____|              | | (_)
%  | |__ _   _ _ __   ___| |_ _  ___  _ __  ___
%  |  __| | | | '_ \ / __| __| |/ _ \| '_ \/ __|
%  | |  | |_| | | | | (__| |_| | (_) | | | \__ \
%  |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/
%
% See: https://patorjk.com/software/taag/#p=display&f=Big&t=Functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%§

function V_unit = normalizeVector(V)
    V_mag = vecnorm(V,2,2);
    V_unit = V./V_mag;
end