% script_test_fcn_Transform_ENUToSensorCoord.m
% tests fcn_Transform_ENUToSensorCoord.m

% Revision history
% 2023_07_27 - Aneesh Batchu

% NOTE: The vehicle and sensor pose parameters can be loaded from
% Example_vehicleParameters_Struct.m

%% Set up the workspace
close all
clc

%% Check assertions for basic path operations and function testing
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                              _   _                 
%      /\                     | | (_)                
%     /  \   ___ ___  ___ _ __| |_ _  ___  _ __  ___ 
%    / /\ \ / __/ __|/ _ \ '__| __| |/ _ \| '_ \/ __|
%   / ____ \\__ \__ \  __/ |  | |_| | (_) | | | \__ \
%  /_/    \_\___/___/\___|_|   \__|_|\___/|_| |_|___/
%                                                    
%                                                    
% See: https://patorjk.com/software/taag/#p=display&f=Big&t=Assertions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Case 1

% Transform matrix to transform ENU point to this coordinates
sensor_or_vehicle = 'vehicle';

% Perturbation in the sensor position and orientation. In this case, the
% perturbation does not matter as we are trying to transform the
% ENU coordinates to vehicle 
perturbation_in_sensorPose_relative_to_SensorPlatform = [0, 0, 0, 0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [2, 0, 0, 0, 0, 0];


transform_Matrix = fcn_Transform_determineTransformMatrix(vehicleParameters, sensorPoseParameters, sensor_or_vehicle, vehiclePose_ENU, perturbation_in_sensorPose_relative_to_SensorPlatform, []);

% This is the expected transform matrix
transform_Matrix_Expec = [
     1     0     0     2;
     0     1     0     0;
     0     0     1     0;
     0     0     0     1];

assert(isequal(transform_Matrix, transform_Matrix_Expec));

%% Case 2

% Transform matrix to transform ENU point to this coordinates
sensor_or_vehicle = 'vehicle';

% Perturbation in the sensor position and orientation. In this case, the
% perturbation does not matter as we are trying to transform the
% ENU coordinates to vehicle 
perturbation_in_sensorPose_relative_to_SensorPlatform = [0, 0, 0, 0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [5, 0, 0, 0, 0, 0];


transform_Matrix = fcn_Transform_determineTransformMatrix(vehicleParameters, sensorPoseParameters, sensor_or_vehicle, vehiclePose_ENU, perturbation_in_sensorPose_relative_to_SensorPlatform, []);


% This is the expected transform matrix
transform_Matrix_Expec = [
     1     0     0     5;
     0     1     0     0;
     0     0     1     0;
     0     0     0     1];

assert(isequal(transform_Matrix, transform_Matrix_Expec));

%% Case 3 

% Transform matrix to transform ENU point to this coordinates
sensor_or_vehicle = 'vehicle';

% Perturbation in the sensor position and orientation. In this case, the
% perturbation does not matter as we are trying to transform the
% ENU coordinates to vehicle 
perturbation_in_sensorPose_relative_to_SensorPlatform = [0, 0, 0, 0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [2, 0, 0, 0, 0, 0; 
                   5, 0, 0, 0, 0, 0;
                   3, 0, 0, 0, 0, 0];


transform_Matrix = fcn_Transform_determineTransformMatrix(vehicleParameters, sensorPoseParameters, sensor_or_vehicle, vehiclePose_ENU, perturbation_in_sensorPose_relative_to_SensorPlatform, []);

% This is the expected transform matrix
transform_Matrix_Expec(:,:,1) = [
     1     0     0     2;
     0     1     0     0;
     0     0     1     0;
     0     0     0     1];

transform_Matrix_Expec(:,:,2) = [
     1     0     0     5;
     0     1     0     0;
     0     0     1     0;
     0     0     0     1];

transform_Matrix_Expec(:,:,3) = [
     1     0     0     3;
     0     1     0     0;
     0     0     1     0;
     0     0     0     1];

assert(isequal(transform_Matrix, transform_Matrix_Expec));

%% Case 4

% Transform matrix to transform ENU point to this coordinates
sensor_or_vehicle = 'vehicle';

% Perturbation in the sensor position and orientation. In this case, the
% perturbation does not matter as we are trying to transform the
% ENU coordinates to vehicle 
perturbation_in_sensorPose_relative_to_SensorPlatform = [0, 0, 0, 0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [2, 1, 5, 0, 0, 0; 
                   5, 9, 2, 0, 0, 0;
                   3, 8, 8, 0, 0, 0];


transform_Matrix = fcn_Transform_determineTransformMatrix(vehicleParameters, sensorPoseParameters, sensor_or_vehicle, vehiclePose_ENU, perturbation_in_sensorPose_relative_to_SensorPlatform, []);

% This is the expected transform matrix
transform_Matrix_Expec(:,:,1) = [
     1     0     0     2;
     0     1     0     1;
     0     0     1     5;
     0     0     0     1];

transform_Matrix_Expec(:,:,2) = [
     1     0     0     5;
     0     1     0     9;
     0     0     1     2;
     0     0     0     1];

transform_Matrix_Expec(:,:,3) = [
     1     0     0     3;
     0     1     0     8;
     0     0     1     8;
     0     0     0     1];

assert(isequal(transform_Matrix, transform_Matrix_Expec));

%% Case 5

% Transform matrix to transform ENU point to this coordinates
sensor_or_vehicle = 'vehicle';

% Perturbation in the sensor position and orientation. In this case, the
% perturbation does not matter as we are trying to transform the
% ENU coordinates to vehicle 
perturbation_in_sensorPose_relative_to_SensorPlatform = [0, 0, 0, 0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0, 0, 0, 0, 0, 45; 
                   5, 9, 2, 0, 0, 0;
                   3, 8, 8, 0, 0, 0];


transform_Matrix = fcn_Transform_determineTransformMatrix(vehicleParameters, sensorPoseParameters, sensor_or_vehicle, vehiclePose_ENU, perturbation_in_sensorPose_relative_to_SensorPlatform, []);

% This is the expected transform matrix
transform_Matrix_Expec(:,:,1) = [
     1     0     0     2;
     0     1     0     1;
     0     0     1     5;
     0     0     0     1];

transform_Matrix_Expec(:,:,2) = [
     1     0     0     5;
     0     1     0     9;
     0     0     1     2;
     0     0     0     1];

transform_Matrix_Expec(:,:,3) = [
     1     0     0     3;
     0     1     0     8;
     0     0     1     8;
     0     0     0     1];

assert(isequal(transform_Matrix, transform_Matrix_Expec));

