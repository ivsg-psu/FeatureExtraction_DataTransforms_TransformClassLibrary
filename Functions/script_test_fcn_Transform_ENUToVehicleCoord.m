% script_test_fcn_Transform_ENUToVehicleCoord.m
%
% This script uses fcn_Transform_ENUToSensorCoord.m to transform the ENU
% coordinates to vehicle coordinates. 
%
% Revision history
% 2023_09_14 - Aneesh Batchu
% -- wrote the code originally

%% Set up the workspace
close all
clc

%% Examples for basic path operations and function testing
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  ______                           _           
% |  ____|                         | |          
% | |__  __  ____ _ _ __ ___  _ __ | | ___  ___ 
% |  __| \ \/ / _` | '_ ` _ \| '_ \| |/ _ \/ __|
% | |____ >  < (_| | | | | | | |_) | |  __/\__ \
% |______/_/\_\__,_|_| |_| |_| .__/|_|\___||___/
%                            | |                
%                            |_|              
%
% See: https://patorjk.com/software/taag/#p=display&f=Big&t=Examples
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Load the example data

run('Example_vehicleParameters_and_sensorPoseParameters_Struct.m')

%% The following examples were written to demonstrate how to transform ENU coordinates to vehicle coordinates using fcn_Transform_ENUToSensorCoord.m

% when a point(sensorReading) in ENU coordinates is transformed to 
% vehicle coordinates

%% Example 1

% The mapping van data is used in this example. The rear left and right GPS
% Antennas's sensor readings(in ENU coordinates) are transformed into
% vehicle coordinates. 

load PoseData.mat

% ENU coordinates of the rear Left Spark Fun GPS 
sensorReading_ENU = GPS_SparkFun_LeftRear_ENU_interp;

% Transforms sensorReading_ENU into Vehicle Coordiantes
in_dashCoord = 'Vehicle';

% vehiclePose_ENU is in PoseData.mat

% Perturbation in the position (in cm) and orientation (in deg) of 
% the velodyne lidar relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
% You cannot plot in this case as the sensorReading_ENU is an array
% Plotting is only possible if sensorReading_ENU is a point
fig_num = [];

% The transformed points in Vehicle Coordinates 
transformed_ENUPoint_in_VehicleCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, ...
    sensorReading_ENU, perturbation_in_sensorPose, fig_num);

disp(transformed_ENUPoint_in_VehicleCoord)

% -0.9995    0.7055    1.5671
%-1.0000    0.7440    1.6006

%% Example 2

% The mapping van data is used in this example. The rear left and right GPS
% Antennas's sensor readings(in ENU coordinates) are transformed into
% vehicle coordinates. 

load PoseData.mat

% ENU coordinates of the rear Left Spark Fun GPS 
sensorReading_ENU = GPS_SparkFun_RightRear_ENU_interp;

% Transforms sensorReading_ENU into Vehicle Coordiantes
in_dashCoord = 'Vehicle';

% vehiclePose_ENU is in PoseData.mat

% Perturbation in the position (in cm) and orientation (in deg) of 
% the velodyne lidar relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
% You cannot plot in this case as the sensorReading_ENU is an array
% Plotting is only possible if sensorReading_ENU is a point
fig_num = [];

% The transformed points in Vehicle Coordinates 
transformed_ENUPoint_in_VehicleCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, ...
    sensorReading_ENU, perturbation_in_sensorPose, fig_num);

disp(transformed_ENUPoint_in_VehicleCoord)

% -0.9995    0.7055    1.5671
% -1.0000   -0.7419    1.6006


%-6.37849

%-6.37991






























