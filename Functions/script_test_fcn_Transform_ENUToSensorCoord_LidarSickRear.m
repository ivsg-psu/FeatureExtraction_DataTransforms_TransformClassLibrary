% script_test_fcn_Transform_ENUToSensorCoord_LidarSickRear.m
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
in_dashCoord = 'Lidar_Sick_Rear';


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

%% Sick Lidar - Case 1

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sick lidar coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,0,0,0,0,0];

% The vehicle's position is at the origin and the orientation of the
% vehicle is also zero which implies -- roll, pitch and yaw are zero
%
% Since we know the offset position of the sick lidar from the sensor 
% platform and how it is oriented, the expected answer of the transformed
% sensorReading_ENU in sick lidar coordinates will be [0 1.54 0.9858]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [0 1.54 0.9858];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% Sick Lidar - Case 2

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sick lidar coordinates.

% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [5,0,0,0,0,0];

% The vehicle is moved 5 units in the X - direction relative to ENU
% coordinates.
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in sick lidar coordinates will be [0 1.54 -4.0142]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [0 1.54 -4.0142];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% Sick Lidar - Case 3

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sick lidar coordinates.

% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,-4,0,0,0,0];

% The vehicle is moved -4 units in the Y - direction relative to ENU
% coordinates.
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in sick lidar coordinates will be [-4 1.54 0.9858]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [-4 1.54 0.9858];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% Sick Lidar - Case 4

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sick lidar coordinates.

% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,0,2,0,0,0];

% The vehicle is moved 2 units in the Z - direction relative to ENU
% coordinates.
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in sick lidar coordinates will be 0 3.54 0.9858]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [0 3.54 0.9858];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,6), round(expected_transformed_ENUPoint_in_dashCoord,6)));

%% Sick Lidar - Case 5

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sick lidar coordinates.

% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [-8,1,-6,0,0,0];

% The vehicle is moved -8 units in the X direction, 1 units in the 
% Y direction, and -6 units in the Z direction relative to ENU coordinates.
% coordinates.
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in sick lidar coordinates will be [1 -4.46 8.9858]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform t
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [1 -4.46 8.9858];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,6), round(expected_transformed_ENUPoint_in_dashCoord,6)));

%% Sick Lidar - Case 6

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sick lidar coordinates.


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
% sensorReading_ENU in sick lidar coordinates will be [-5 2.54 9.9858]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [-5 2.54 9.9858];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,6), round(expected_transformed_ENUPoint_in_dashCoord,6)));

%% Sick Lidar - Case 7

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sick lidar coordinates.

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
% sensorReading_ENU in sick lidar coordinates will be [7 2.54 -3.0142]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [7 2.54 -3.0142];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,6), round(expected_transformed_ENUPoint_in_dashCoord,6)));

%% Sick Lidar - Case 8

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sick lidar coordinates.

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
% sensorReading_ENU in sick lidar coordinates will be [3 8.54 9.9858]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [3 8.54 9.9858];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,6), round(expected_transformed_ENUPoint_in_dashCoord,6)));

%% Sick Lidar - Case 9

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sick lidar coordinates.

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
% sensorReading_ENU in sick lidar coordinates will be [3 5.54 3.9858]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [3 5.54 3.9858];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,6), round(expected_transformed_ENUPoint_in_dashCoord,6)));

%% Sick Lidar - Case 10

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sick lidar coordinates.

% A point in ENU coordinates. 
sensorReading_ENU = [3, 4, -5];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,0,0,0,0,0];

% The vehicle's position is at the origin and the orientation of the
% vehicle is also zero which implies -- roll, pitch and yaw are zero
%
% Since we know the offset position of the sick lidar from the sensor 
% platform and how it is oriented, the expected answer of the transformed
% sensorReading_ENU in sick lidar coordinates will be [-4 6.54 3.9858]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [-4 6.54 3.9858];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,6), round(expected_transformed_ENUPoint_in_dashCoord,6)));

%% Sick Lidar - Case 11

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sick lidar coordinates.


% A point in ENU coordinates. 
sensorReading_ENU = [1, -6, 8];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [5,0,0,0,0,0];

% The vehicle is moved 5 units in the X - direction relative to ENU
% coordinates.
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in sick lidar coordinates will be [-11.5, -6, 2.4]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [6 -6.46 -3.0142];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,6), round(expected_transformed_ENUPoint_in_dashCoord,6)));

%% Sick Lidar - Case 12

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sick lidar coordinates.


% A point in ENU coordinates. 
sensorReading_ENU = [-4, 2, 1];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,-4,0,0,0,0];

% The vehicle is moved -4 units in the Y - direction relative to ENU
% coordinates.
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in sick lidar coordinates will be [-6 0.54 -3.0142]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [-6 0.54 -3.0142];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,6), round(expected_transformed_ENUPoint_in_dashCoord,6)));

%% Sick Lidar - Case 13

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sick lidar coordinates.


% A point in ENU coordinates. 
sensorReading_ENU = [2, 6, -3];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,0,2,0,0,0];

% The vehicle is moved 2 units in the Z - direction relative to ENU
% coordinates.
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in sick lidar coordinates will be [-6 6.54 2.9858]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform  
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [-6 6.54 2.9858];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,6), round(expected_transformed_ENUPoint_in_dashCoord,6)));

%% Sick Lidar - Case 14

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sick lidar coordinates.


% A point in ENU coordinates.
sensorReading_ENU = [1, 4, 3];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [-8,1,-6,0,0,0];

% The vehicle is moved -8 units in the X direction, 1 units in the 
% Y direction, and -6 units in the Z direction relative to ENU coordinates.
% coordinates.
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in sick lidar coordinates will be [6.5, 3, 8.4]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [-3 -7.46 9.9858];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,6), round(expected_transformed_ENUPoint_in_dashCoord,6)));

%% Sick Lidar - Case 15

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sick lidar coordinates.


% A point in ENU coordinates.
sensorReading_ENU = [-6, 2, 7];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [5,-9,1,0,0,90];

% The vehicle is moved 5 units in the X direction, -9 units in the 
% Y direction, and 1 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 0, 0, and 90, respectively
% relative to ENU coordinates. 
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in sick lidar coordinates will be [-11 -4.46 11.9858]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [-11 -4.46 11.9858];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,6), round(expected_transformed_ENUPoint_in_dashCoord,6)));

%% Sick Lidar - Case 16

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sick lidar coordinates.

% A point in ENU coordinates. 
sensorReading_ENU = [3, 6, 1];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [1,7,-4,0,90,0];

% The vehicle is moved 1 units in the X direction, 7 units in the 
% Y direction, and -4 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 0, 90, and 0, respectively
% relative to ENU coordinates. 
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in sick lidar coordinates will be [1 -0.46 -4.0142]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform  
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [1 -0.46 -4.0142];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,6), round(expected_transformed_ENUPoint_in_dashCoord,6)));

%% Sick Lidar - Case 17

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sick lidar coordinates.


% A point in ENU coordinates. 
sensorReading_ENU = [3, 9, 8];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [-9,-7,3,90,0,0];

% The vehicle is moved -9 units in the X direction, -7 units in the 
% Y direction, and 3 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 90, 0, and 0, respectively
% relative to ENU coordinates. 
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in sick lidar coordinates will be [-5 17.54 12.9858]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform  
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [-5 17.54 12.9858];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,6), round(expected_transformed_ENUPoint_in_dashCoord,6)));

%% Sick Lidar - Case 18

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sick lidar coordinates.


% A point in ENU coordinates. 
sensorReading_ENU = [8, 1, -3];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [3,4,-3,180,270,90];

% The vehicle is moved 3 units in the X direction, 4 units in the 
% Y direction, and -3 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 180, 270, and 90, respectively
% relative to ENU coordinates. 
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in sick lidar coordinates will be [-5 4.54 0.9858]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in Sick Lidar Rear
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [-5 4.54 0.9858];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,6), round(expected_transformed_ENUPoint_in_dashCoord,6)));

%% Sick Lidar with perturbations - Case 19 

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sick lidar coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,0,0,0,0,0];

% The vehicle's position is at the origin and the orientation of the
% vehicle is also zero which implies -- roll, pitch and yaw are zero
%
% Since we know the offset position of the sick lidar from the sensor 
% platform and how it is oriented, the expected answer of the transformed
% sensorReading_ENU in sick lidar coordinates will be [0 1.54 -0.0142]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform 
perturbation_in_sensorPose = [100, 0, 0, 0, 0, 0];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [0 1.54 -0.0142];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,6), round(expected_transformed_ENUPoint_in_dashCoord,6)));

%% Sick Lidar with perturbations - Case 20

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sick lidar coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,0,0,0,0,0];

% The vehicle's position is at the origin and the orientation of the
% vehicle is also zero which implies -- roll, pitch and yaw are zero
%
% Since we know the offset position of the sick lidar from the sensor 
% platform and how it is oriented, the expected answer of the transformed
% sensorReading_ENU in sick lidar coordinates will be [0.05 1.74 -0.0142]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform 
perturbation_in_sensorPose = [100, 5, 20, 0, 0, 0];


% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [0.05 1.74 -0.0142];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,6), round(expected_transformed_ENUPoint_in_dashCoord,6)));

%% Sick Lidar with perturbations - Case 21

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sick lidar coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,0,0,0,0,0];

% The vehicle's position is at the origin and the orientation of the
% vehicle is also zero which implies -- roll, pitch and yaw are zero
%
% Since we know the offset position of the sick lidar from the sensor 
% platform and how it is oriented, the expected answer of the transformed
% sensorReading_ENU in sick lidar coordinates will be [-1.54 0 0.9858]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform  
perturbation_in_sensorPose = [0, 0, 0, 0, -90, 0];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [-1.54 0 0.9858];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,6), round(expected_transformed_ENUPoint_in_dashCoord,6)));
