% script_test_fcn_Transform_ENUToSensorCoord_vehicle.m
% tests fcn_Transform_ENUToSensorCoord.m
%
% This script uses fcn_Transform_ENUToSensorCoord.m to transform the ENU
% coordinates to vehicle coordinates. 
%
% Revision history
% 2023_09_25 - Aneesh Batchu
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
in_dashCoord = 'Vehicle';


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

%% Case - 1

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [5,0,0,0,0,0]; 

% The vehicle is moved 5 units in the X - direction relative to ENU
% coordinates. Therefore, the expected answer of the transformed
% sensorReading_ENU in vehicle coordinates will be [-5, 0, 0]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'vehicleCoord';  

% Perturbation in sensor Pose 
% We are transforming the point to vehicle coordinates. We cannot add
% perturbation to it
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [-5, 0, 0];
 
assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% Vehicle Coordinates - Case 2

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,-4,0,0,0,0];

% The vehicle is moved -4 units in the Y - direction relative to ENU
% coordinates. Therefore, the expected answer of the transformed
% sensorReading_ENU in vehicle coordinates will be [0, 4, 0]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'vehicle';  

% Perturbation in sensor Pose 
% We are transforming the point to vehicle coordinates. We cannot add
% perturbation to it
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [0, 4, 0];
 
assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% Vehicle Coordinates - Case 3

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,0,2,0,0,0];

% The vehicle is moved 2 units in the Z - direction relative to ENU
% coordinates. Therefore, the expected answer of the transformed
% sensorReading_ENU in vehicle coordinates will be [0, 0, -2];

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'vehicle_coord';  

% Perturbation in sensor Pose 
% We are transforming the point to vehicle coordinates. We cannot add
% perturbation to it
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [0, 0, -2];
 
assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% Vehicle Coordinates - Case 4 - vehilclePose_ENU with roll, pitch and yaw = 0.

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [-5,7,3,0,0,0];

% The vehicle is moved -5 units in the X direction, 7 units in the 
% Y direction, and 3 units in the Z direction relative to ENU coordinates.
% coordinates. Therefore, the expected answer of the transformed
% sensorReading_ENU in vehicle coordinates will be [5, -7, -3];

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'vehicle';  

% Perturbation in sensor Pose 
% We are transforming the point to vehicle coordinates. We cannot add
% perturbation to it
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [5, -7, -3];
 
assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% Vehicle Coordinates - Case 5 - vehilclePose_ENU with yaw

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [4,5,-6,0,0,90];

% The vehicle is moved 4 units in the X direction, 5 units in the 
% Y direction, and -6 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 0, 0, and 90, respectively
% relative to ENU coordinates. 
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in vehicle coordinates will be [-5, 4, 6];

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'vehicle';  

% Perturbation in sensor Pose 
% We are transforming the point to vehicle coordinates. We cannot add
% perturbation to it
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [-5, 4, 6];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% Vehicle Coordinates - Case 6 - vehilclePose_ENU with pitch

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [4,5,-6,0,90,0];

% The vehicle is moved 4 units in the X direction, 5 units in the 
% Y direction, and -6 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 0, 90, and 0, respectively
% relative to ENU coordinates. 
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in vehicle coordinates will be [-6, -5, -4];

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'vehicle';  

% Perturbation in sensor Pose 
% We are transforming the point to vehicle coordinates. We cannot add
% perturbation to it
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [-6, -5, -4];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% Vehicle Coordinates - Case 7 - vehilclePose_ENU with position and roll 

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [4,5,-6,-90,0,0];

% The vehicle is moved 4 units in the X direction, 5 units in the 
% Y direction, and -6 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 90, 0, and 0, respectively
% relative to ENU coordinates. 
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in vehicle coordinates will be [-4, 6, 5];

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'vehicle';  

% Perturbation in sensor Pose 
% We are transforming the point to vehicle coordinates. We cannot add
% perturbation to it
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [-4, -6, -5];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% Vehicle Coordinates - Case 8 - vehilclePose_ENU with position, pitch and yaw

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [1,-9,5,0,90,180];

% The vehicle is moved 1 units in the X direction, -9 units in the 
% Y direction, and 5 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 0, 90, and 180, respectively
% relative to ENU coordinates. 
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in vehicle coordinates will be [5, -9, 1]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'vehicle';  

% Perturbation in sensor Pose 
% We are transforming the point to vehicle coordinates. We cannot add
% perturbation to it
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [5, -9, 1];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% Vehicle Coordinates - Case 9 - vehilclePose_ENU with position, roll and pitch

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [1,-9,5,-90,180,0];

% The vehicle is moved 1 units in the X direction, -9 units in the 
% Y direction, and 5 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 0, 90, and 180, respectively
% relative to ENU coordinates. 
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in vehicle coordinates will be [1, -5, 9]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'vehicle_coordinates';  

% Perturbation in sensor Pose 
% We are transforming the point to vehicle coordinates. We cannot add
% perturbation to it
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [1, -5, 9];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% Vehicle Coordinates - Case 10 - vehilclePose_ENU with position, roll and yaw

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [1,-9,5,-180,0,90];

% The vehicle is moved 1 units in the X direction, -9 units in the 
% Y direction, and 5 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 180, 0, and 90, respectively
% relative to ENU coordinates. 
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in vehicle coordinates will be [9, -1, 5]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'vehicle_coordinates';  

% Perturbation in sensor Pose 
% We are transforming the point to vehicle coordinates. We cannot add
% perturbation to it
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [9, -1, 5];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% Vehicle Coordinates - Case 11 - vehilclePose_ENU with position, roll, pitch and yaw

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [1,-9,5,-270,90,180];

% The vehicle is moved 1 units in the X direction, -9 units in the 
% Y direction, and 5 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 270, 90, and 180, respectively
% relative to ENU coordinates. 
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in vehicle coordinates will be [5, 1, 9]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'vehicle_coordinates';  

% Perturbation in sensor Pose 
% We are transforming the point to vehicle coordinates. We cannot add
% perturbation to it
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [5, 1, 9];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% Vehicle Coordinates - Case 12 - vehilclePose_ENU with position, and roll, pitch and yaw greater than 360 degrees

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [1,-9,5,-(360+270),720+90,1080+180];

% The vehicle is moved 1 units in the X direction, -9 units in the 
% Y direction, and 5 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 360+270, 720+90, and 1080+180, respectively
% relative to ENU coordinates. 
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in vehicle coordinates will be [5, 1, 9]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'vehicle_coordinates';  

% Perturbation in sensor Pose 
% We are transforming the point to vehicle coordinates. We cannot add
% perturbation to it
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
vexpected_transformed_ENUPoint_in_dashCoord = [5, 1, 9];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,4), round(expected_transformed_ENUPoint_in_dashCoord,4)));

%% Vehicle Coordinates - Case 13 - vehilclePose_ENU with position, and negative roll, pitch and yaw

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [1,-9,5,270,-90,-180];

% The vehicle is moved 1 units in the X direction, -9 units in the 
% Y direction, and 5 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 270, 90, and 180, respectively
% relative to ENU coordinates. 
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in vehicle coordinates will be [-5, 1, -9]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'vehicle_coordinates';  

% Perturbation in sensor Pose 
% We are transforming the point to vehicle coordinates. We cannot add
% perturbation to it
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [-5, 1, -9];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% Vehicle Coordinates - Case 14 - When sensorReading_ENU is not [0, 0, 0].

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. 
sensorReading_ENU = [5, -3, 2];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [5,0,0,0,0,0]; 

% The vehicle is moved 5 units in the X - direction relative to ENU
% coordinates. Therefore, the expected answer of the transformed
% sensorReading_ENU in vehicle coordinates will be [0, -3, 2]. The expected
% answer can be guessed from sensorReading_ENU = [0, 0, 0] cases.

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'vehicleCoord';  

% Perturbation in sensor Pose 
% We are transforming the point to vehicle coordinates. We cannot add
% perturbation to it
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [0, -3, 2];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% Vehicle Coordinates - Case 15 - When sensorReading_ENU is not [0, 0, 0].

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. 
sensorReading_ENU = [-7, -2, -5];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,8,0,0,0,0]; 

% The vehicle is moved 8 units in the Y - direction relative to ENU
% coordinates. Therefore, the expected answer of the transformed
% sensorReading_ENU in vehicle coordinates will be [-7, -10, -5]. 
% The expected answer can be guessed from sensorReading_ENU = [0, 0, 0] 
% cases.

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'vehicle Coord';  

% Perturbation in sensor Pose 
% We are transforming the point to vehicle coordinates. We cannot add
% perturbation to it
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [-7, -10,-5];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% Vehicle Coordinates - Case 16 - When sensorReading_ENU is not [0, 0, 0].

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. 
sensorReading_ENU = [4, 6, -4];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,0,-6,0,0,0]; 

% The vehicle is moved -6 units in the Z - direction relative to ENU
% coordinates. Therefore, the expected answer of the transformed
% sensorReading_ENU in vehicle coordinates will be [4, 6, 2]. The expected
% answer can be guessed from sensorReading_ENU = [0, 0, 0] cases.

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'vehicle Coord';  

% Perturbation in sensor Pose 
% We are transforming the point to vehicle coordinates. We cannot add
% perturbation to it
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [4, 6, 2];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% Vehicle Coordinates - Case 17 - When sensorReading_ENU is not [0, 0, 0].

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. 
sensorReading_ENU = [2, -1, 9];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [-4,7,2,0,0,0]; 

% The vehicle is moved -4 units in the X direction, 7 units in the 
% Y direction, and 2 units in the Z direction relative to ENU coordinates.
% coordinates. Therefore, the expected answer of the transformed
% sensorReading_ENU in vehicle coordinates will be [6, -8, 7]. The expected
% answer can be guessed from sensorReading_ENU = [0, 0, 0] cases.

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'vehicle Coord';  

% Perturbation in sensor Pose 
% We are transforming the point to vehicle coordinates. We cannot add
% perturbation to it
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [6, -8, 7];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% Vehicle Coordinates - Case 18 - When sensorReading_ENU is not [0, 0, 0].

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% % A point in ENU coordinates. 
% sensorReading_ENU = [2, -1, 9];
% 
% % vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
% vehiclePose_ENU = [-4,7,2,0,0,90]; 

% A point in ENU coordinates. 
sensorReading_ENU = 1e3*[2, -1, 9];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = 1e3*[-4,7,2,0,0,0.090]; 

% The vehicle is moved -4 units in the X direction, 7 units in the 
% Y direction, and 2 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 0, 0, and 90, respectively
% relative to ENU coordinates. Therefore, the expected answer of the 
% transformed sensorReading_ENU in vehicle coordinates will be [-8, -6, 7]. 
% The expected answer can be guessed from sensorReading_ENU = [0, 0, 0] 
% cases.

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'vehicle Coord';  

% Perturbation in sensor Pose 
% We are transforming the point to vehicle coordinates. We cannot add
% perturbation to it
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = 1e3*[-8, -6, 7];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% Vehicle Coordinates - Case 19 - When sensorReading_ENU is not [0, 0, 0].

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. 
sensorReading_ENU = [2, -1, 9];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [-4,7,2,0,90,0]; 

% The vehicle is moved -4 units in the X direction, 7 units in the 
% Y direction, and 2 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 0, 90, and 0, respectively
% relative to ENU coordinates. Therefore, the expected answer of the 
% transformed sensorReading_ENU in vehicle coordinates will be [-7, -8, 6]. 
% The expected answer can be guessed from sensorReading_ENU = [0, 0, 0] 
% cases.

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'vehicle Coord';  

% Perturbation in sensor Pose 
% We are transforming the point to vehicle coordinates. We cannot add
% perturbation to it
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [-7, -8, 6];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% Vehicle Coordinates - Case 20 - When sensorReading_ENU is not [0, 0, 0].

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. 
sensorReading_ENU = [2, -1, 9];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [-4,7,2,-90,0,0]; 

% The vehicle is moved -4 units in the X direction, 7 units in the 
% Y direction, and 2 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 90, 0, and 0, respectively
% relative to ENU coordinates. Therefore, the expected answer of the 
% transformed sensorReading_ENU in vehicle coordinates will be [6, -7, -8]. 
% The expected answer can be guessed from sensorReading_ENU = [0, 0, 0] 
% cases.

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'vehicle Coord';  

% Perturbation in sensor Pose 
% We are transforming the point to vehicle coordinates. We cannot add
% perturbation to it
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [6, -7, -8];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% Vehicle Coordinates - Case 21 - When sensorReading_ENU is not [0, 0, 0].

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. 
sensorReading_ENU = [2, -1, 9];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [-4,7,2,-90,180,270]; 

% The vehicle is moved -4 units in the X direction, 7 units in the 
% Y direction, and 2 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 90, 180, and 270, respectively
% relative to ENU coordinates. Therefore, the expected answer of the 
% transformed sensorReading_ENU in vehicle coordinates will be [-8, 7, 6]. 
% The expected answer can be guessed from sensorReading_ENU = [0, 0, 0] 
% cases.

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'vehicle Coord';  

% Perturbation in sensor Pose 
% We are transforming the point to vehicle coordinates. We cannot add
% perturbation to it
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [-8, 7, 6];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));
