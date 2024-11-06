function v_x_vehicle_unit_mean = fcn_Transform_constructOrthonormalBasis_VehicleXaxis(GPS_SparkFun_Front_ENU,...
                                                                                        GPS_SparkFun_LeftRear_ENU,...
                                                                                        GPS_SparkFun_RightRear_ENU)
% fcn_Transform_constructOrthonormalBasis_VehicleXaxis takes GPS_data_struct as input 
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

%% Step 2 - Select the rear right GPS antenna as the origin


%% Step 3: Calculate the midpoint of the two rear GPS antennas, define it as the origin of the GPS coordiante system
O_midpoint_ENU = (GPS_SparkFun_LeftRear_ENU+GPS_SparkFun_RightRear_ENU)/2;
%% Step 4: Calculate two vectors point from the origin to the left and front GPS antenna
v_origin_left = GPS_SparkFun_LeftRear_ENU - O_midpoint_ENU; % a vector point from the origin to the left GPS antenna

%% Step 5: Compute vectors that form the basis 
% Average three GPS antennas moving trajectories, use the average vector as the X axis of the vehicle coordinate system
v_front_traj = fcn_Calibration_CalculateGPSTrajectory(GPS_SparkFun_Front_ENU);
v_left_traj = fcn_Calibration_CalculateGPSTrajectory(GPS_SparkFun_LeftRear_ENU);
v_right_traj = fcn_Calibration_CalculateGPSTrajectory(GPS_SparkFun_RightRear_ENU);
v_x_vehicle = mean(rmoutliers([v_front_traj;v_left_traj;v_right_traj]),1);
v_x_vehicle_unit = normalizeVector(v_x_vehicle); % identify the direction X-axis of Vehicle Coordinate Frame
v_x_vehicle_unit_clean = v_x_vehicle_unit;
v_x_vehicle_unit_mean = mean(v_x_vehicle_unit_clean,1);


end
%% Normalize vector to unit vector
function V_unit = normalizeVector(V)
    V_mag = vecnorm(V,2,2);
    V_unit = V./V_mag;
end