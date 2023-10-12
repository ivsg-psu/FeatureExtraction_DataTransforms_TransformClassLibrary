% script_test_fcn_Transform_generateVehiclePoseDuplicates.m
% tests fcn_Transform_generateVehiclePoseDuplicates.m

% Revision history
% 2023_10_12 - Aneesh Batchu
% -- wrote the code originally

%% Set up the workspace

clc
close all

%% Examples for basic path operations and function testing
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  ______                           _
% |  ____|                         | |
% | |__  __  ____ _ _ __ ___  _ __ | | ___  ___
% |  __| \ \/ / _` | '_ ` _ \| '_ \| |/ _ \/ __|
% | |____ >  < (_| | | | | | | |_) | |  __/\__ \
% |______/_/\_\__,_|_| |_| |_| .__/|_|\___||___/
%                            | |
%                            |_|
% See: https://patorjk.com/software/taag/#p=display&f=Big&t=Examples
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Example - 1

% In this example, the sensor data has 10 elements and the vehicle pose has
% 2 elements. Therefore, the vehiclePose_DuplicatesUpdated will have 10
% elements. Each element in vehicle pose is duplicated 5 times
% consecutively. 

% Assume sensor and vehicle pose data
SensorData = rand(10,3);
vehiclePose = rand(2,6);

vehiclePose_DuplicatesUpdated = fcn_Transform_generateVehiclePoseDuplicates(vehiclePose, SensorData);

disp(vehiclePose_DuplicatesUpdated)

%% Example - 2

% In this example, the sensor data has 10 elements and the vehicle pose has
% 4 elements. Therefore, the vehiclePose_DuplicatesUpdated will have 10
% elements. Except the last element, each element in vehicle pose is 
% repeated 3 times consecutively

% Assume sensor and vehicle pose data
SensorData = rand(10,3);
vehiclePose = rand(4,6);

vehiclePose_DuplicatesUpdated = fcn_Transform_generateVehiclePoseDuplicates(vehiclePose, SensorData);

disp(vehiclePose_DuplicatesUpdated)

%% Example - 2

% In this example the size(,1) of both sensor data and vehicle pose data is
% same. Therefore, the size(,1) of the vehiclePose_DuplicatesUpdated does
% does not change and will be same as original vehicle pose data

% Assume sensor and vehicle pose data
SensorData = rand(3,3);
vehiclePose = rand(3,6);

vehiclePose_DuplicatesUpdated = fcn_Transform_generateVehiclePoseDuplicates(vehiclePose, SensorData);

disp(vehiclePose_DuplicatesUpdated)


