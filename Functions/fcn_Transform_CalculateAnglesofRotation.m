function [roll_array,pitch_array, yaw_array] = fcn_Transform_CalculateAnglesofRotation(GPS_SparkFun_LeftRear_ENU,GPS_SparkFun_RightRear_ENU,GPS_SparkFun_Front_ENU,calibrate_matrix)
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
%       GPS_SparkFun_LeftRear_ENU:  a Nx3 vectors of points containing the
%       ENU coordinates of the Rear Left SparkFun GPS
%
%       GPS_SparkFun_RightRear_ENU: a Nx3 vectors of points containing the
%       ENU coordinates of the Rear Right SparkFun GPS
%
%       GPS_SparkFun_Front_ENU: a Nx3 vectors of points containing the
%       ENU coordinates of the Front SparkFun GPS
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


% Input

[~, ~,V_GPS_virtual_frame] = fcn_Transform_calculateYawAndPitchOffset(rawdata_1);

% The ISO convention is used for vehicle-affixed coordinate system
% Assume the Rear Left and Rear Right GPS Antennas are perfect aligned
% along the vehicle y axis

V_right_to_left = GPS_SparkFun_LeftRear_ENU - GPS_SparkFun_RightRear_ENU; % Nx3
V_right_to_left_mag = vecnorm(V_right_to_left,2,2); % Nx3
V_right_to_left_unit = V_right_to_left./V_right_to_left_mag; % Nx3

% The front GPS antenna is not aligned with any rear antenna along the
% vehicle x axis, so we used vector projection to create a new virtual
% front GPS antenna
V_right_to_front = GPS_SparkFun_Front_ENU - GPS_SparkFun_RightRear_ENU; % Nx3
V_projection = fcn_Transform_VectorProjection(V_right_to_front, V_right_to_left_unit); % The projection is along V_y_unit, Nx3
V_right_to_virtual = V_right_to_front - V_projection; % Nx3
V_right_to_virtual_mag = vecnorm(V_right_to_virtual,2,2); % Nx3
V_right_to_virtual_unit = V_right_to_virtual./V_right_to_virtual_mag;

%% GPS Frame
V_x_GPS_frame = V_GPS_virtual_frame.X;
V_y_GPS_frame = V_GPS_virtual_frame.Y;
% V_z_GPS_frame = V_GPS_virtual_frame.Z;



%% Correct the angel
V_x_unit_transpose = calibrate_matrix\V_x_GPS_frame.';
V_y_unit_transpose = calibrate_matrix\V_y_GPS_frame.';
V_x_unit = V_x_unit_transpose.';
V_y_unit = V_y_unit_transpose.';
V_z_unit = cross(V_x_unit,V_y_unit,2);
% angle_90 = rad2deg(asin(vecnorm(cross(V_x_unit,V_right_to_left_unit,2),2,2)));

N_points = length(V_x_unit);
for n = 1:N_points
    R = [V_x_unit(n,:).' V_y_unit(n,:).' V_z_unit(n,:).'];
    roll = atan2(R(3,2),R(3,3));
    pitch = atan2(-R(3,1),sqrt(R(3,2)^2+R(3,3)^2));
    yaw = atan2(R(2,1), R(1,1));
    roll_array(n,1) = roll;
    pitch_array(n,1) = pitch;
    yaw_array(n,1) = yaw;

end

