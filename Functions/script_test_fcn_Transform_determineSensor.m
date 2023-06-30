% script_test_fcn_Transform_determineSensor.m
% tests fcn_Transform_determineSensor.m

% Revision history
% 2023_06_21 - Aneesh Batchu
% -- wrote the code originally

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

%% Basic test - 'sick' is 'sicklidar'
type_of_sensor = 'sick';
sensor = fcn_Transform_determineSensor(type_of_sensor);
assert(strcmp(sensor,'sicklidar'));

%% Basic test - 'GPS_right' is 'rightGPS'
type_of_sensor = 'GPS_right';
sensor = fcn_Transform_determineSensor(type_of_sensor);
assert(strcmp(sensor,'rightGPS'));

%% Basic test - 'VelodyneLidar1' is 'velodynelidar'
type_of_sensor = 'VelodyneLidar1';
sensor = fcn_Transform_determineSensor(type_of_sensor);
assert(strcmp(sensor,'velodynelidar'));

%% Basic test - 'LeftSidegps' is 'leftGPS'
type_of_sensor = 'LeftSidegps';
sensor = fcn_Transform_determineSensor(type_of_sensor);
assert(strcmp(sensor,'leftGPS'));

%% Basic test - 'sensor_platform' is 'sensorplatform'
type_of_sensor = 'sensor_platform';
sensor = fcn_Transform_determineSensor(type_of_sensor);
assert(strcmp(sensor,'sensorplatform'));

%% Basic test - 'AlltheSensors12345' is 'allsensors'
type_of_sensor = 'AlltheSensors12345';
sensor = fcn_Transform_determineSensor(type_of_sensor);
assert(strcmp(sensor,'allsensors'));

%% Basic test - 'AlltheSensors12345' is 'allsensors'
type_of_sensor = 'VEhiCLE_Coordinates';
sensor = fcn_Transform_determineSensor(type_of_sensor);
assert(strcmp(sensor,'vehicle'));
%% Fail conditions 

%% ERROR for wrong/unclear inputs
if 1==0
    type_of_sensor = "lidar";
    sensor = fcn_Transform_determineSensor(type_of_sensor);
    assert(strcmp(sensor,'sicklidar'));
end
    %% ERROR for wrong/unclear inputs
if 1==0
    type_of_sensor = "GPS";
    sensor = fcn_Transform_determineSensor(type_of_sensor);
    assert(strcmp(sensor,'leftGPS'));
end


