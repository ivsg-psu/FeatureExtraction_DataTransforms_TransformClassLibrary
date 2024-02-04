function [yaw_offset_array, pitch_offset_array, roll_offset_array] = fcn_Transform_CalculateAnglesOffsets(rawdata_1,rawdata_2,process_range)
% fcn_Transform_CalculateAngleBetweenVectors calculates the angle in rad
% between two vectors
% two vectors given as V_1 and V_2
%
% FORMAT:
%
% [angle, angle_deg] = fcn_Transform_CalculateAngleBetweenVectors(V_1, V_2,(fig_num))
%
% INPUTS:
%
%      V_1: a 1x3 vector
%
%      V_2: a 1x3 vector
%
%      (OPTIONAL INPUTS)
%
%      fig_num: a figure number to plot results. If set to -1, skips any
%      input checking or debugging, no figures will be generated, and sets
%      up code to maximize speed.
%
% OUTPUTS:
%
%      angle: the angle between two vectors, as a scaler, from V_1 to V_2
%
%
% DEPENDENCIES:
%
%      fcn_DebugTools_checkInputsToFunctions
%      fcn_geometry_plotCircle
%
% EXAMPLES:
%      
%      % BASIC example
%      points = [0 0; 1 4; 0.5 -1];
%      [centers,radii] = fcn_geometry_circleCenterFrom3Points(points,1)
% 
% See the script: script_test_fcn_Transform_CalculateAngleBetweenVectors
% for a full test suite.
%
% This function was written on 2023_10_20 by X.Cao
% Questions or comments? xfc5113@psu.edu

% Revision history:
% 2023_10_20 - wrote the code
% 2024_01_28 - added more comments, particularly to explain inputs more
% clearly


[yaw_offset_array_1, pitch_offset_array_1,V_virtual_frame_1] = fcn_Transform_calculateYawAndPitchOffset(rawdata_1,process_range);
[yaw_offset_array_2, pitch_offset_array_2,V_virtual_frame_2] = fcn_Transform_calculateYawAndPitchOffset(rawdata_2,process_range);
yaw_offset_array = [yaw_offset_array_1;yaw_offset_array_2];
pitch_offset_array = [pitch_offset_array_1;pitch_offset_array_2];
V_y_virtual_1 = V_virtual_frame_1.Y;
V_y_virtual_2 = V_virtual_frame_2.Y;
%% Roll

roll_offset_array = fcn_Transform_calculateRollOffset(V_y_virtual_1,V_y_virtual_2);