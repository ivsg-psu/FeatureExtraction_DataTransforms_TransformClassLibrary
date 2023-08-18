% script_test_fcn_Transform_determineSensorTypeOrVehicle.m
% tests fcn_Transform_determineSensorTypeOrVehicle.m

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
type_of_sensor = 'rear_sick_lidar';
sensor = fcn_Transform_determineSensorTypeOrVehicle(type_of_sensor);
assert(strcmp(sensor,'sicklidarrear'));

%% Basic test - 'GPS_right' is 'rightGPS'
type_of_sensor = 'GPS_right_rear';
sensor = fcn_Transform_determineSensorTypeOrVehicle(type_of_sensor);
assert(strcmp(sensor,'gpssparkfunrightrear'));

%% Basic test - 'VelodyneLidar1' is 'velodynelidar'
type_of_sensor = 'VelodynerearLidar1';
sensor = fcn_Transform_determineSensorTypeOrVehicle(type_of_sensor);
assert(strcmp(sensor,'velodynelidarrear'));

%% Basic test - 'LeftSidegps' is 'leftGPS'
type_of_sensor = 'LeftSidegpsREAR';
sensor = fcn_Transform_determineSensorTypeOrVehicle(type_of_sensor);
assert(strcmp(sensor,'gpssparkfunleftrear'));

%% Basic test - 'sensor_platform' is 'sensorplatform'
type_of_sensor = 'sensor_platform_gps_SparkFun_rear';
sensor = fcn_Transform_determineSensorTypeOrVehicle(type_of_sensor);
assert(strcmp(sensor,'gpssensorplatformrear'));

%% Basic test - 'AlltheSensors12345' is 'allsensors'
type_of_sensor = 'AlltheSensors12345';
sensor = fcn_Transform_determineSensorTypeOrVehicle(type_of_sensor);
assert(strcmp(sensor,'allsensors'));

%% Basic test - 'VEhiCLE_Coordinates' is 'vehicle'
type_of_sensor = 'VEhiCLE_Coordinates';
sensor = fcn_Transform_determineSensorTypeOrVehicle(type_of_sensor);
assert(strcmp(sensor,'vehicle'));

%% Fail conditions 

%% ERROR for wrong/unclear inputs
if 1==0
    type_of_sensor = "lidar";
    sensor = fcn_Transform_determineSensorTypeOrVehicle(type_of_sensor);
    assert(strcmp(sensor,'sicklidarrear'));
end
    %% ERROR for wrong/unclear inputs
if 1==0
    type_of_sensor = "GPS";
    sensor = fcn_Transform_determineSensorTypeOrVehicle(type_of_sensor);
    assert(strcmp(sensor,'gpssparkfunleftrear'));
end


