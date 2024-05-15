function calibrate_matrix = fcn_Transform_createCalibrateMatrix(rawdata_1,rawdata_2,process_range,ref_basestation)

% fcn_Transform_calculateYawAndPitchOffset takes GPS_data_struct as input 
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
%% Step 1 - Preprocess the GPS data, all unlocked data will be removed and time will be synchronized (Temp version, will update later)
LidarType = 'none';
[GPS_Data_Struct_Output_1,~,~,~] = fcn_GPSDataPreprocess(rawdata_1,LidarType,ref_basestation);
[GPS_Data_Struct_Output_2,~,~,~] = fcn_GPSDataPreprocess(rawdata_2,LidarType,ref_basestation);


%% Step 2 - Select the process range
% [GPS_Data_Struct_Output,traj_range] = fcn_GPS_Calibration_selectProcessRange(GPS_Data_Struct, process_range)
% if process_range == 'all'
%     GPS_SparkFun_Front_ENU_selected = GPS_SparkFun_Front_ENU_interp;
%     GPS_SparkFun_LeftRear_ENU_selected = GPS_SparkFun_LeftRear_ENU_interp;
%     GPS_SparkFun_RightRear_ENU_selected = GPS_SparkFun_RightRear_ENU_interp;
%     traj_range = 1:(length(GPS_SparkFun_Front_ENU_selected)+1);
% else
%     GPS_SparkFun_Front_ENU_selected = GPS_SparkFun_Front_ENU_interp(process_range,:);
%     GPS_SparkFun_LeftRear_ENU_selected = GPS_SparkFun_LeftRear_ENU_interp(process_range,:);
%     GPS_SparkFun_RightRear_ENU_selected = GPS_SparkFun_RightRear_ENU_interp(process_range,:);
%     traj_range = 1:(length(process_range)+1);
% end
GPS_SparkFun_Front_ENU_1 =  GPS_Data_Struct_Output_1.GPS_SparkFun_Front_ENU;
GPS_SparkFun_LeftRear_ENU_1 = GPS_Data_Struct_Output_1.GPS_SparkFun_LeftRear_ENU;
GPS_SparkFun_RightRear_ENU_1 = GPS_Data_Struct_Output_1.GPS_SparkFun_RightRear_ENU;

[g_x_unit_1,g_y_unit_1,g_z_unit_1] = fcn_Transform_constructOrthonormalBasis_GPSCoordinateSystem(GPS_SparkFun_Front_ENU_1,GPS_SparkFun_LeftRear_ENU_1,GPS_SparkFun_RightRear_ENU_1);

