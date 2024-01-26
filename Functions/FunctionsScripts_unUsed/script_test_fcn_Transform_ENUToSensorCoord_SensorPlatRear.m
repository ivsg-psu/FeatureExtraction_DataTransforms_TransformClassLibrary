% script_test_fcn_Transform_ENUToSensorCoord_GPSSensorPlatformRear.m
% tests fcn_Transform_ENUToSensorCoord.m
%
% This script uses fcn_Transform_ENUToSensorCoord.m to transform the ENU
% coordinates to vehicle coordinates. 
%
% Revision history
% 2023_09_27 - Aneesh Batchu
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

%% Example 1

% The mapping van data is used in this example. The rear left and right GPS
% Antennas's sensor readings(in ENU coordinates) are transformed into
% vehicle coordinates.

% Load the mat file
load PoseData_1.mat

% ENU data of the GPS_SparkFun_LeftRear
LeftGPS_ENU = GPS_SparkFun_LeftRear_ENU_interp ;

% ENU data of the GPS_SparkFun_RightRear
RightGPS_ENU = GPS_SparkFun_RightRear_ENU_interp;

% Pitch of the vehicle is assumed as zero
PITCH_vehicle_ENU = zeros(size(LeftGPS_ENU,1),1);

% figure(1)
% plot3(LeftGPS_ENU(:,1),LeftGPS_ENU(:,2),LeftGPS_ENU(:,3),'b-','Linewidth',3);
% hold on
% grid on
% box on
% plot3(RightGPS_ENU(:,1),RightGPS_ENU(:,2),RightGPS_ENU(:,3),'r-','Linewidth',3);


% The sensor mount offset relative to vehicle origin = 
% [-X_SensorMount_center, 0, +Z_SensorMount_center] in meters
SensorMount_offset_relative_to_VehicleOrigin = [-0.6027 0 1.5261]; 

% vehiclePose of the vehicle is determined as the data in PoseData_1 is
% not accurate
vehiclePose_ENU = fcn_Transform_findVehiclePoseinENU(LeftGPS_ENU, RightGPS_ENU, PITCH_vehicle_ENU, SensorMount_offset_relative_to_VehicleOrigin);


vehiclePose_ENU_pos = vehiclePose_ENU(:,1:3);

% Plot the vehicle position
% plot3(vehiclePose_ENU_pos(:,1),vehiclePose_ENU_pos(:,2),vehiclePose_ENU_pos(:,3),'k-','Linewidth',3);


% ENU coordinates of the rear Left Spark Fun GPS 
sensorReading_ENU = GPS_SparkFun_LeftRear_ENU_interp;

% Transforms sensorReading_ENU into Vehicle Coordiantes
in_dashCoord = 'GPS_SensorPlatform_Rear';


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


%% Sensor Platform - Case 1

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sensor platform coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,0,0,0,0,0];

% The vehicle's position is at the origin and the orientation of the
% vehicle is also zero which implies -- roll, pitch and yaw are zero
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in sensor platform coordinates will be [0.6027 0 -1.5261]
% which is the -1*position of the sensor platform with respect to 
% vehicle coordinates.


% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_Hemisphere_SensorPlatform_Rear'; 

% Perturbation in the position (in cm) and orientation (in deg) of 
% the sensor platform relative to vehicle body 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [0.6027 0 -1.5261];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));


%% Sensor Platform - Case 2

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sensor platform coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [5,0,0,0,0,0];

% The vehicle is moved 5 units in the X - direction relative to ENU
% coordinates.
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in sensor platform coordinates will be [-4.3973 0 -1.5261]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_Hemisphere_SensorPlatform_Rearwr';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the sensor platform relative to vehicle body 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [-4.3973 0 -1.5261];

% assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

assert(isequal(round(transformed_ENUPoint_in_dashCoord,6), round(expected_transformed_ENUPoint_in_dashCoord,6)));

%% Sensor Platform - Case 3

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sensor platform coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,-4,0,0,0,0];

% The vehicle is moved -4 units in the Y - direction relative to ENU
% coordinates.
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in sensor platform coordinates will be [0.6027 4 -1.5261]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_Hemisphere_SensorPlatform_Rear354';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the sensor platform relative to vehicle body 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [0.6027 4 -1.5261];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% Sensor Platform - Case 4

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sensor platform coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,0,2,0,0,0];

% The vehicle is moved 2 units in the Z - direction relative to ENU
% coordinates.
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in sensor platform coordinates will be [0.6027 0 -3.5261]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPSw4_Hemisphere_SensorPlatform_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the sensor platform relative to vehicle body 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [0.6027 0 -3.5261];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% Sensor Platform - Case 5

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sensor platform coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [-8,1,-6,0,0,0];

% The vehicle is moved -8 units in the X direction, 1 units in the 
% Y direction, and -6 units in the Z direction relative to ENU coordinates.
% coordinates.
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in sensor platform coordinates will be [8.6027   -1.0000    4.4739]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_Hemisphere_SensorPlatform_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the sensor platform relative to vehicle body 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [8.6027 -1 4.4739];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% Sensor Platform - Case 6

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sensor platform coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [5,-9,1,0,0,90];

% The vehicle is moved 5 units in the X direction, -9 units in the 
% Y direction, and 1 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 0, 0, and 90, respectively
% relative to ENU coordinates. 
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in sensor platform coordinates will be [9.6027 5 -2.5261]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_Hemisphere_SensorPlatform_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the sensor platform relative to vehicle body 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [9.6027 5 -2.5261];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% Sensor Platform - Case 7

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sensor platform coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [1,7,-4,0,90,0];

% The vehicle is moved 1 units in the X direction, 7 units in the 
% Y direction, and -4 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 0, 90, and 0, respectively
% relative to ENU coordinates. 
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in sensor platform coordinates will be [-3.3973 -7 -2.5261]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_Hemisphere_SensorPlatform_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the sensor platform relative to vehicle body 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [-3.3973 -7 -2.5261];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% Sensor Platform - Case 8

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sensor platform coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [-9,-7,3,90,0,0];

% The vehicle is moved -9 units in the X direction, -7 units in the 
% Y direction, and 3 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 90, 0, and 0, respectively
% relative to ENU coordinates. 
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in sensor platform coordinates will be [9.6027 -3 -8.5261]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_Hemisphere_SensorPlatform_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the sensor platform relative to vehicle body 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [9.6027 -3 -8.5261];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% Sensor Platform - Case 9

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sensor platform coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [3,4,-3,180,270,90];

% The vehicle is moved 3 units in the X direction, 4 units in the 
% Y direction, and -3 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 180, 270, and 90, respectively
% relative to ENU coordinates. 
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in sensor platform coordinates will be [3.6027 -3 -5.5261]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_Hemisphere_SensorPlatform_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the sensor platform relative to vehicle body 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [3.6027 -3 -5.5261];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% Sensor Platform - Case 10

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sensor platform coordinates.


% A point in ENU coordinates. 
sensorReading_ENU = [5, 3, -2];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,0,0,0,0,0];

% The vehicle's position is at the origin and the orientation of the
% vehicle is also zero which implies -- roll, pitch and yaw are zero
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in sensor platform coordinates will be [5.6027 3 -3.5261]
% which is the -1*position of the sensor platform with respect to 
% vehicle coordinates.


% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_Hemisphere_SensorPlatform_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the sensor platform relative to vehicle body 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [5.6027 3 -3.5261];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,6), round(expected_transformed_ENUPoint_in_dashCoord,6)));


%% Sensor Platform - Case 11

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sensor platform coordinates.


% A point in ENU coordinates. 
sensorReading_ENU = [1, -3, 6];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [5,0,0,0,0,0];

% The vehicle is moved 5 units in the X - direction relative to ENU
% coordinates.
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in sensor platform coordinates will be [-3.3973 -3 4.4739]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_Hemisphere_SensorPlatform_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the sensor platform relative to vehicle body 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [-3.3973 -3 4.4739];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,6), round(expected_transformed_ENUPoint_in_dashCoord,6)));

%% Sensor Platform - Case 12

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sensor platform coordinates.


% A point in ENU coordinates. 
sensorReading_ENU = [-4, 7, -1];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,-4,0,0,0,0];

% The vehicle is moved -4 units in the Y - direction relative to ENU
% coordinates.
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in sensor platform coordinates will be [-3.3973 11 -2.5261]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_Hemisphere_SensorPlatform_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the sensor platform relative to vehicle body 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [-3.3973 11 -2.5261];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% Sensor Platform - Case 13

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sensor platform coordinates.


% A point in ENU coordinates. 
sensorReading_ENU = [7, 2, -4];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,0,2,0,0,0];

% The vehicle is moved 2 units in the Z - direction relative to ENU
% coordinates.
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in sensor platform coordinates will be [7.6027 2 -7.5261]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_Hemisphere_SensorPlatform_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the sensor platform relative to vehicle body 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [7.6027 2 -7.5261];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,6), round(expected_transformed_ENUPoint_in_dashCoord,6)));

%% Sensor Platform - Case 14

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sensor platform coordinates.


% A point in ENU coordinates. 
sensorReading_ENU = [2, -7, 11];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [-8,1,-6,0,0,0];

% The vehicle is moved -8 units in the X direction, 1 units in the 
% Y direction, and -6 units in the Z direction relative to ENU coordinates.
% coordinates.
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in sensor platform coordinates will be [10.6027 -8 15.4739]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_Hemisphere_SensorPlatform_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the sensor platform relative to vehicle body 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [10.6027 -8 15.4739];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% Sensor Platform - Case 15

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sensor platform coordinates.

% A point in ENU coordinates. 
sensorReading_ENU = [1, -2, 3];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [5,-9,1,0,0,90];

% The vehicle is moved 5 units in the X direction, -9 units in the 
% Y direction, and 1 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 0, 0, and 90, respectively
% relative to ENU coordinates. 
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in sensor platform coordinates will be [7.6027 4 0.4739]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_Hemisphere_SensorPlatform_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the sensor platform relative to vehicle body 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [7.6027 4 0.4739];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,6), round(expected_transformed_ENUPoint_in_dashCoord,6)));

%% Sensor Platform - Case 16

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sensor platform coordinates.

% A point in ENU coordinates. 
sensorReading_ENU = [-2, 8, 1];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [1,7,-4,0,90,0];

% The vehicle is moved 1 units in the X direction, 7 units in the 
% Y direction, and -4 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 0, 90, and 0, respectively
% relative to ENU coordinates. 
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in sensor platform coordinates will be [-4.3973 1 -4.5261]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = '   GPS_Hemisphere_SensorPlatform_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the sensor platform relative to vehicle body 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [-4.3973 1 -4.5261];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,6), round(expected_transformed_ENUPoint_in_dashCoord,6)));

%% Sensor Platform - Case 17

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sensor platform coordinates.

% A point in ENU coordinates. 
sensorReading_ENU = [5, 1, 2];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [-9,-7,3,90,0,0];

% The vehicle is moved -9 units in the X direction, -7 units in the 
% Y direction, and 3 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 90, 0, and 0, respectively
% relative to ENU coordinates. 
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in sensor platform coordinates will be [14.6027 -1 -9.5261]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_Hemisphere_SensorPlatform_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the sensor platform relative to vehicle body 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [14.6027 -1 -9.5261];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,6), round(expected_transformed_ENUPoint_in_dashCoord,6)));

%% Sensor Platform - Case 18

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sensor platform coordinates.

% A point in ENU coordinates. 
sensorReading_ENU = [1, 1, 3];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [3,4,-3,180,270,90];

% The vehicle is moved 3 units in the X direction, 4 units in the 
% Y direction, and -3 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 180, 270, and 90, respectively
% relative to ENU coordinates. 
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in sensor platform coordinates will be [6.6027 -2 -4.5261]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_Hemisphere_SensorPlatform_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the sensor platform relative to vehicle body 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [6.6027 -2 -4.5261];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,6), round(expected_transformed_ENUPoint_in_dashCoord,6)));
