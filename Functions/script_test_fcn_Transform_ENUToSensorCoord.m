% script_test_fcn_Transform_ENUToSensorCoord.m
% tests fcn_Transform_ENUToSensorCoord.m

% Revision history
% 2023_06_29 - Aneesh Batchu
% -- wrote the code originally
% 2023_07_23 - Aneesh Batchu
% -- added cases for the sensors with perturbation in their position and
% orientation


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


%% Load the example data

run('Example_vehicleParameters_and_sensorPoseParameters_Struct.m')


%% The following cases were written to test the accuracy of vehicle coordinates 

% when a point(sensorReading) in ENU coordinates is transformed to 
% vehicle coordinates


%% Vehicle Coordinates - Case 1

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

%% Vehicle Coordinates - Case 4

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. 
sensorReading_ENU = [1, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [5,0,0,0,0,0]; 

% The vehicle is moved 5 units in the X - direction relative to ENU
% coordinates. Therefore, the expected answer of the transformed
% sensorReading_ENU in vehicle coordinates will be [-4, 0, 0]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'vehicle coordinates';  

% Perturbation in sensor Pose 
% We are transforming the point to vehicle coordinates. We cannot add
% perturbation to it
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [-4, 0, 0];
 
assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% Vehicle Coordinates - Case 5

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. 
sensorReading_ENU = [1, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [5,0,0,0,0,90]; 

% The vehicle is moved 5 units in the X - direction relative to ENU
% coordinates. Therefore, the expected answer of the transformed
% sensorReading_ENU in vehicle coordinates will be [0, 4, 0]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'vehicle_Coord';  

% Perturbation in sensor Pose 
% We are transforming the point to vehicle coordinates. We cannot add
% perturbation to it
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [0, 4, 0];
 
assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% Vehicle Coordinates - Case 6

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. 
sensorReading_ENU = [1, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [5,0,0,0,90,0]; 

% The vehicle is moved 5 units in the X - direction relative to ENU
% coordinates. Therefore, the expected answer of the transformed
% sensorReading_ENU in vehicle coordinates will be [0, 0, -4]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'Coordinates_vehicle';  

% Perturbation in sensor Pose 
% We are transforming the point to vehicle coordinates. We cannot add
% perturbation to it
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [0, 0, -4];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% Vehicle Coordinates - Case 7

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. 
sensorReading_ENU = [1, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [5,0,0,90,0,0]; 

% The vehicle is moved 5 units in the X - direction relative to ENU
% coordinates. Therefore, the expected answer of the transformed
% sensorReading_ENU in vehicle coordinates will be [0, 0, -4]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'vehicle';  

% Perturbation in sensor Pose 
% We are transforming the point to vehicle coordinates. We cannot add
% perturbation to it
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [-4, 0, 0];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% Vehicle Coordinates - Case 8 - vehilclePose_ENU with roll, pitch and yaw = 0.

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

%% Vehicle Coordinates - Case 9 - vehilclePose_ENU with yaw

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

%% Vehicle Coordinates - Case 10 - vehilclePose_ENU with pitch

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

%% Vehicle Coordinates - Case 11 - vehilclePose_ENU with position and roll 

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [4,5,-6,90,0,0];

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

%% Vehicle Coordinates - Case 12 - vehilclePose_ENU with position, pitch and yaw

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

%% Vehicle Coordinates - Case 13 - vehilclePose_ENU with position, roll and pitch

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [1,-9,5,90,180,0];

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

%% Vehicle Coordinates - Case 14 - vehilclePose_ENU with position, roll and yaw

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [1,-9,5,180,0,90];

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

%% Vehicle Coordinates - Case 15 - vehilclePose_ENU with position, roll, pitch and yaw

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [1,-9,5,270,90,180];

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

%% Vehicle Coordinates - Case 16 - vehilclePose_ENU with position, and roll, pitch and yaw greater than 360 degrees

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [1,-9,5,360+270,720+90,1080+180];

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

%% Vehicle Coordinates - Case 17 - vehilclePose_ENU with position, and negative roll, pitch and yaw

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [1,-9,5,-270,-90,-180];

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

%% Vehicle Coordinates - Case 18 - When sensorReading_ENU is not [0, 0, 0].

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

%% Vehicle Coordinates - Case 19 - When sensorReading_ENU is not [0, 0, 0].

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

%% Vehicle Coordinates - Case 20 - When sensorReading_ENU is not [0, 0, 0].

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

%% Vehicle Coordinates - Case 21 - When sensorReading_ENU is not [0, 0, 0].

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

%% Vehicle Coordinates - Case 22 - When sensorReading_ENU is not [0, 0, 0].

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
expected_transformed_ENUPoint_in_dashCoord = [-8, -6, 7];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% Vehicle Coordinates - Case 23 - When sensorReading_ENU is not [0, 0, 0].

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

%% Vehicle Coordinates - Case 24 - When sensorReading_ENU is not [0, 0, 0].

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. 
sensorReading_ENU = [2, -1, 9];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [-4,7,2,90,0,0]; 

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

%% Vehicle Coordinates - Case 25 - When sensorReading_ENU is not [0, 0, 0].

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. 
sensorReading_ENU = [2, -1, 9];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [-4,7,2,90,180,270]; 

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

%% Vehicle Coordinates - Case 26 - When sensorReading_ENU is not [0, 0, 0].

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into vehicle coordinates.


% A point in ENU coordinates. 
sensorReading_ENU = [2, -1, 9];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [-4,7,2,0,0,0]; 

% The vehicle is moved -4 units in the X direction, 7 units in the 
% Y direction, and 2 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 360, 360, and 360, respectively
% relative to ENU coordinates. Therefore, the expected answer of the 
% transformed sensorReading_ENU in vehicle coordinates will be [6, -8, 7]. 
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
expected_transformed_ENUPoint_in_dashCoord = [6, -8, 7];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));


%% The following cases were written to test the accuracy of sensor platform coordinates 

% when a point(sensorReading) in ENU coordinates is transformed to 
% sensor platform coordinates


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
% sensorReading_ENU in sensor platform coordinates will be [1, 0, -1.6]
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
expected_transformed_ENUPoint_in_dashCoord = [1, 0, -1.6];

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
% sensorReading_ENU in sensor platform coordinates will be [-4, 0, -1.6]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_Hemisphere_SensorPlatform_Rearwr';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the sensor platform relative to vehicle body 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [-4, 0, -1.6];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

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
% sensorReading_ENU in sensor platform coordinates will be [1, 4, -1.6]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_Hemisphere_SensorPlatform_Rear354';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the sensor platform relative to vehicle body 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [1, 4, -1.6];

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
% sensorReading_ENU in sensor platform coordinates will be [1, 0, -3.6]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPSw4_Hemisphere_SensorPlatform_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the sensor platform relative to vehicle body 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [1, 0, -3.6];

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
% sensorReading_ENU in sensor platform coordinates will be [9, -1, 4.4]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_Hemisphere_SensorPlatform_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the sensor platform relative to vehicle body 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [9, -1, 4.4];

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
% sensorReading_ENU in sensor platform coordinates will be [9, 4, -2.6]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_Hemisphere_SensorPlatform_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the sensor platform relative to vehicle body 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [9, 4, -2.6];

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
% sensorReading_ENU in sensor platform coordinates will be [-2.4, -7, 0]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_Hemisphere_SensorPlatform_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the sensor platform relative to vehicle body 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [-2.4, -7, 0];

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
% sensorReading_ENU in sensor platform coordinates will be [10, 4.6, 7]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_Hemisphere_SensorPlatform_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the sensor platform relative to vehicle body 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [10, 4.6, 7];

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
% sensorReading_ENU in sensor platform coordinates will be [1.4, -2, -4]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_Hemisphere_SensorPlatform_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the sensor platform relative to vehicle body 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [1.4, -2, -4];

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
% sensorReading_ENU in sensor platform coordinates will be [6, 3, -3.6]
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
expected_transformed_ENUPoint_in_dashCoord = [6, 3, -3.6];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));


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
% sensorReading_ENU in sensor platform coordinates will be [-3, -3, 4.4]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_Hemisphere_SensorPlatform_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the sensor platform relative to vehicle body 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [-3, -3, 4.4];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

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
% sensorReading_ENU in sensor platform coordinates will be [-3, 11, -2.6]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_Hemisphere_SensorPlatform_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the sensor platform relative to vehicle body 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [-3, 11, -2.6];

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
% sensorReading_ENU in sensor platform coordinates will be [8, 2, -7.6]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_Hemisphere_SensorPlatform_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the sensor platform relative to vehicle body 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = 8768;

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [8, 2, -7.6];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

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
% sensorReading_ENU in sensor platform coordinates will be [11, -8, 15.4]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_Hemisphere_SensorPlatform_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the sensor platform relative to vehicle body 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [11, -8, 15.4];

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
% sensorReading_ENU in sensor platform coordinates will be [7, 3, 0.4]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_Hemisphere_SensorPlatform_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the sensor platform relative to vehicle body 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = 4356;

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [7, 3, 0.4];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,4), round(expected_transformed_ENUPoint_in_dashCoord,4)));

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
% sensorReading_ENU in sensor platform coordinates will be [-3.4, 1, -2]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = '   GPS_Hemisphere_SensorPlatform_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the sensor platform relative to vehicle body 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [-3.4, 1, -2];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

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
% sensorReading_ENU in sensor platform coordinates will be [15, 2.6, 8]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_Hemisphere_SensorPlatform_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the sensor platform relative to vehicle body 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [15, 2.6, 8];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,4), round(expected_transformed_ENUPoint_in_dashCoord,4)));

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
% sensorReading_ENU in sensor platform coordinates will be [4.4, -1, -3]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_Hemisphere_SensorPlatform_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the sensor platform relative to vehicle body 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [4.4, -1, -3];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

%% The following cases were written to test the accuracy of sick lidar coordinates 

% when a point(sensorReading) in ENU coordinates is transformed to 
% sick lidar coordinates


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
% sensorReading_ENU in sick lidar coordinates will be [1.5, 0, 1.4]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [1.5, 0, 1.4];

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
% sensorReading_ENU in sick lidar coordinates will be [-3.5, 0, 1.4]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [-3.5, 0, 1.4];

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
% sensorReading_ENU in sick lidar coordinates will be [1.5, 4, 1.4]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [1.5, 4, 1.4];

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
% sensorReading_ENU in sick lidar coordinates will be [1.5, 0, -0.6]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [1.5, 0, -0.6];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,4), round(expected_transformed_ENUPoint_in_dashCoord,4)));

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
% sensorReading_ENU in sick lidar coordinates will be [9.5, -1, 7.4]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform t
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [9.5, -1, 7.4];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

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
% sensorReading_ENU in sick lidar coordinates will be [9, 3.5, 0.4]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = 6789098;

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [9, 3.5, 0.4];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,4), round(expected_transformed_ENUPoint_in_dashCoord,4)));

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
% sensorReading_ENU in sick lidar coordinates will be [-5.4, -7, 0.5]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [-5.4, -7, 0.5];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

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
% sensorReading_ENU in sick lidar coordinates will be [10.5, 1.6, 7]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [10.5, 1.6, 7];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

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
% sensorReading_ENU in sick lidar coordinates will be [4.4, -1.5, -4]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [4.4, -1.5, -4];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

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
% sensorReading_ENU in sick lidar coordinates will be [6.5, 4, 4.4]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [6.5, 4, 4.4];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

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
expected_transformed_ENUPoint_in_dashCoord = [-11.5, -6, 2.4];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

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
% sensorReading_ENU in sick lidar coordinates will be [0.5, 6, -2.6]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [0.5, 6, -2.6];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

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
% sensorReading_ENU in sick lidar coordinates will be [4.5, 6, 1.4]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform  
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [4.5, 6, 1.4];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,4), round(expected_transformed_ENUPoint_in_dashCoord,4)));

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
expected_transformed_ENUPoint_in_dashCoord = [6.5, 3, 8.4];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

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
% sensorReading_ENU in sick lidar coordinates will be [11, 10.5, -5.6]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [11, 10.5, -5.6];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,4), round(expected_transformed_ENUPoint_in_dashCoord,4)));

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
% sensorReading_ENU in sick lidar coordinates will be [-8.4, -1, -0.5]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform  
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [-8.4, -1, -0.5];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

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
% sensorReading_ENU in sick lidar coordinates will be [2.5, -1.4, 16]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform  
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [2.5, -1.4, 16];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

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
% sensorReading_ENU in sick lidar coordinates will be [12.4, 1.5, -3]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in Sick Lidar Rear
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [12.4, 1.5, -3];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

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
% sensorReading_ENU in sick lidar coordinates will be [1.5, 0, 0.4]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform 
perturbation_in_sensorPose = [100, 0, 0, 0, 0, 0];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [1.5, 0, 0.4];

assert(isequal(transformed_ENUPoint_in_dashCoord, expected_transformed_ENUPoint_in_dashCoord));

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
% sensorReading_ENU in sick lidar coordinates will be [1.7, -0.05, 0.4]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform 
perturbation_in_sensorPose = [100, 5, 20, 0, 0, 0];


% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [1.7, -0.05, 0.4];

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
% sensorReading_ENU in sick lidar coordinates will be [1.4, 0, -1.5]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform  
perturbation_in_sensorPose = [0, 0, 0, 0, -90, 0];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [1.4, 0, -1.5];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,6), round(expected_transformed_ENUPoint_in_dashCoord,6)));

%% The following cases were written to test the accuracy of velodyne lidar coordinates 

% when a point(sensorReading) in ENU coordinates is transformed to 
% velodyne lidar coordinates


%% Velodyne Lidar - Case 1

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into velodyne lidar coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,0,0,0,0,0];

% The vehicle's position is at the origin and the orientation of the
% vehicle is also zero which implies -- roll, pitch and yaw are zero
%
% Since the sensorReading_ENU and vehiclePose_ENU is [0, 0, 0] and 
% [0,0,0,0,0,0], this gives the pose of the velodyne Lidar.
% Therefore, the expected answer of the transformed
% sensorReading_ENU in velodyne lidar coordinates will be [0, -0.4425, -2.7936]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'Lidar_Velodyne_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the velodyne lidar relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = 656;

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [0, -0.442463, -2.793604];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,6), round(expected_transformed_ENUPoint_in_dashCoord,6)));

%% Velodyne Lidar - Case 2

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into velodyne lidar coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [5,0,0,0,0,0];

% The vehicle is moved 5 units in the X - direction relative to ENU
% coordinates.
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in velodyne lidar coordinates will be [-5, -0.4425, -2.7936]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'Lidar_Velodyne_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the velodyne lidar relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [-5, -0.442463, -2.793604];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,6), round(expected_transformed_ENUPoint_in_dashCoord,6)));

%% Velodyne Lidar - Case 3

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into velodyne lidar coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,-4,0,0,0,0];

% The vehicle is moved -4 units in the Y - direction relative to ENU
% coordinates.
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in velodyne lidar coordinates will be [0, 3.5575, -2.7936]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'Lidar_Velodyne_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the velodyne lidar relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [0, -0.442463 + 4, -2.793604];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,6), round(expected_transformed_ENUPoint_in_dashCoord,6)));

%% Velodyne Lidar - Case 4

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into velodyne lidar coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,0,2,0,0,0];

% The vehicle is moved 2 units in the Z - direction relative to ENU
% coordinates.
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in velodyne lidar coordinates will be [0, -0.4425, -4.7936]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'Lidar_Velodyne_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the velodyne lidar relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [0, -0.442463, -2.793604 - 2];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,6), round(expected_transformed_ENUPoint_in_dashCoord,6)));

%% Velodyne Lidar - Case 5

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into velodyne lidar coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [-8,1,-6,0,0,0];

% The vehicle is moved -8 units in the X direction, 1 units in the 
% Y direction, and -6 units in the Z direction relative to ENU coordinates.
% coordinates.
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in velodyne lidar coordinates will be [0, -0.4425, -4.7936]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'Lidar_Velodyne_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the velodyne lidar relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [0 + 8, -0.442463 - 1, -2.793604 + 6];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,6), round(expected_transformed_ENUPoint_in_dashCoord,6)));

%% Velodyne Lidar - Case 6

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into velodyne lidar coordinates.


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
% sensorReading_ENU in velodyne lidar coordinates will be [8.5575, 5, -3.7936]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'Lidar_Velodyne_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the velodyne lidar relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = 134;

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
disp(transformed_ENUPoint_in_dashCoord)
% expected_transformed_ENUPoint_in_dashCoord = [8.5575, 5, -3.7936];
% 
% assert(isequal(round(transformed_ENUPoint_in_dashCoord,4), round(expected_transformed_ENUPoint_in_dashCoord,4)));

%% Velodyne Lidar - Case 7

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into velodyne lidar coordinates.


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
% sensorReading_ENU in velodyne lidar coordinates will be [-1.2064, -7.4425, -1]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'Lidar_Velodyne_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the velodyne lidar relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [-1.2064, -7.4425, -1];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,4), round(expected_transformed_ENUPoint_in_dashCoord,4)));

%% Velodyne Lidar - Case 8

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into velodyne lidar coordinates.


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
% sensorReading_ENU in velodyne lidar coordinates will be [9, 5.7936, 6.5575]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'Lidar_Velodyne_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the velodyne lidar relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [9, 5.7936, 6.5575];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,4), round(expected_transformed_ENUPoint_in_dashCoord,4)));

%% Velodyne Lidar - Case 9

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into velodyne lidar coordinates.


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
% sensorReading_ENU in velodyne lidar coordinates will be [0.2064, -3, -4.4425]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'Lidar_Velodyne_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the velodyne lidar relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [0.2064, -3, -4.4425];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,4), round(expected_transformed_ENUPoint_in_dashCoord,4)));

%% The following cases were written to test the accuracy of left GPS coordinates 

% when a point(sensorReading) in ENU coordinates is transformed to 
% left GPS coordinates


%% Left GPS - Case 1

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into left GPS coordinates.

% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,0,0,0,0,0];

% The vehicle's position is at the origin and the orientation of the
% vehicle is also zero which implies -- roll, pitch and yaw are zero
%
% Since we know the offset position of the left GPS from the sensor 
% platform and how it is oriented, the expected answer of the transformed
% sensorReading_ENU in left GPS coordinates will be [0.7, -0.762, -2.1]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_SparkFun_LeftRear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear left GPS relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [0.7, -0.762, -2.1];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,4), round(expected_transformed_ENUPoint_in_dashCoord,4)));

%% Left GPS - Case 2

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into left GPS coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [5,0,0,0,0,0];

% The vehicle is moved 5 units in the X - direction relative to ENU
% coordinates.
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in left GPS coordinates will be [-4.3, -0.762, -2.1]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_SparkFun_LeftRear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear left GPS relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [-4.3, -0.762, -2.1];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,4), round(expected_transformed_ENUPoint_in_dashCoord,4)));

%% Left GPS - Case 3

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into left GPS coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,-4,0,0,0,0];

% The vehicle is moved -4 units in the Y - direction relative to ENU
% coordinates.
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in left GPS coordinates will be [0.7, 3.238, -2.1]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_SparkFun_LeftRear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear left GPS relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [0.7, 3.238, -2.1];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,4), round(expected_transformed_ENUPoint_in_dashCoord,4)));

%% Left GPS - Case 4

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into left GPS coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,0,2,0,0,0];

% The vehicle is moved 2 units in the Z - direction relative to ENU
% coordinates.
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in left GPS coordinates will be [0.7, -0.762, -4.1]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_SparkFun_LeftRear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear left GPS relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [0.7, -0.762, -4.1];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,4), round(expected_transformed_ENUPoint_in_dashCoord,4)));

%% Left GPS - Case 5

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into left GPS coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [-8,1,-6,0,0,0];

% The vehicle is moved -8 units in the X direction, 1 units in the 
% Y direction, and -6 units in the Z direction relative to ENU coordinates.
% coordinates.
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in left GPS coordinates will be [8.7, -1.762, 3.9]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'leftGPS_SparkFun_LeftReargps';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear left GPS relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [8.7, -1.762, 3.9];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,4), round(expected_transformed_ENUPoint_in_dashCoord,4)));

%% Left GPS - Case 6

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into left GPS coordinates.


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
% sensorReading_ENU in left GPS coordinates will be [8.238, 4.3, -3.1]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_SparkFun_LeftRear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear left GPS relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [8.238, 4.3, -3.1];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,4), round(expected_transformed_ENUPoint_in_dashCoord,4)));

%% Left GPS - Case 7

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into left GPS coordinates.


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
% sensorReading_ENU in left GPS coordinates will be [-1.9, -7.762, -0.3]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_SparkFun_LeftRear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear left GPS relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [-1.9, -7.762, -0.3];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,4), round(expected_transformed_ENUPoint_in_dashCoord,4)));

%% Left GPS - Case 8

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into left GPS coordinates.


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
% sensorReading_ENU in left GPS coordinates will be [9.7, 5.1, 6.238]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_SparkFun_LeftRear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear left GPS relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [9.7, 5.1, 6.238];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,4), round(expected_transformed_ENUPoint_in_dashCoord,4)));

%% Left GPS - Case 9

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into left GPS coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [3,4,-3,180,270,90];

% The vehicle is moved 3 units in the X direction, 4 units in the 
% Y direction, and -3 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 180, 270, and 90, respectively
% relative to ENU coordinate
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in left GPS coordinates will be [0.9, -2.3, -4.762]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_SparkFun_LeftRear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear left GPS relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [0.9, -2.3, -4.762];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,4), round(expected_transformed_ENUPoint_in_dashCoord,4)));

%% Left GPS - Case 10

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into left GPS coordinates.

% A point in ENU coordinates. 
sensorReading_ENU = [6, -4, 9];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,0,0,0,0,0];

% The vehicle's position is at the origin and the orientation of the
% vehicle is also zero which implies -- roll, pitch and yaw are zero
%
% Since we know the offset position of the left GPS from the sensor 
% platform and how it is oriented, the expected answer of the transformed
% sensorReading_ENU in left GPS coordinates will be [6.7, -4.762, 6.9]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_SparkFun_LeftRear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear left GPS relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [6.7, -4.762, 6.9];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,4), round(expected_transformed_ENUPoint_in_dashCoord,4)));

%% Left GPS - Case 11

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into left GPS coordinates.


% A point in ENU coordinates. 
sensorReading_ENU = [12, 4, -1];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [5,0,0,0,0,0];

% The vehicle is moved 5 units in the X - direction relative to ENU
% coordinates.
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in left GPS coordinates will be [7.7, 3.238, -3.1]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_SparkFun_LeftRear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear left GPS relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [7.7, 3.238, -3.1];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,4), round(expected_transformed_ENUPoint_in_dashCoord,4)));

%% Left GPS - Case 12

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into left GPS coordinates.


% A point in ENU coordinates. 
sensorReading_ENU = [-8, 9, 3];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,-4,0,0,0,0];

% The vehicle is moved -4 units in the Y - direction relative to ENU
% coordinates.
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in left GPS coordinates will be [-7.3, 12.238, 0.9]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_SparkFun_LeftRear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear left GPS relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [-7.3, 12.238, 0.9];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,4), round(expected_transformed_ENUPoint_in_dashCoord,4)));

%% Left GPS - Case 13

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into left GPS coordinates.


% A point in ENU coordinates. 
sensorReading_ENU = [6, 2, 9];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,0,2,0,0,0];

% The vehicle is moved 2 units in the Z - direction relative to ENU
% coordinates.
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in left GPS coordinates will be [6.7, 1.238, 4.9]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_SparkFun_LeftRear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear left GPS relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [6.7, 1.238, 4.9];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,4), round(expected_transformed_ENUPoint_in_dashCoord,4)));

%% Left GPS - Case 14

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into left GPS coordinates.


% A point in ENU coordinates. 
sensorReading_ENU = [11, -5, 4];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [-8,1,-6,0,0,0];

% The vehicle is moved -8 units in the X direction, 1 units in the 
% Y direction, and -6 units in the Z direction relative to ENU coordinates.
% coordinates.
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in left GPS coordinates will be [19.7, -6.762, 7.9]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_SparkFun_LeftRear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear left GPS relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [19.7, -6.762, 7.9];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,4), round(expected_transformed_ENUPoint_in_dashCoord,4)));

%% Left GPS - Case 15

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into left GPS coordinates.


% A point in ENU coordinates. 
sensorReading_ENU = [5, -1, 9];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [5,-9,1,0,0,90];

% The vehicle is moved 5 units in the X direction, -9 units in the 
% Y direction, and 1 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 0, 0, and 90, respectively
% relative to ENU coordinates. 
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in left GPS coordinates will be [7.238, -0.7, 5.9]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_SparkFun_LeftRear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear left GPS relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [7.238, -0.7, 5.9];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,4), round(expected_transformed_ENUPoint_in_dashCoord,4)));

%% Left GPS - Case 16

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into left GPS coordinates.


% A point in ENU coordinates. 
sensorReading_ENU = [-9, 2, 8];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [1,7,-4,0,90,0];

% The vehicle is moved 1 units in the X direction, 7 units in the 
% Y direction, and -4 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 0, 90, and 0, respectively
% relative to ENU coordinates.
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in left GPS coordinates will be [-9.9, -5.762, -9.3]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_SparkFun_LeftRear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear left GPS relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [-9.9, -5.762, -9.3];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,4), round(expected_transformed_ENUPoint_in_dashCoord,4)));

%% Left GPS - Case 17

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into left GPS coordinates.


% A point in ENU coordinates. 
sensorReading_ENU = [6, -3, 1];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [-9,-7,3,90,0,0];

% The vehicle is moved -9 units in the X direction, -7 units in the 
% Y direction, and 3 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 90, 0, and 0, respectively
% relative to ENU coordinates. 
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in left GPS coordinates will be [15.7, 4.1, 3.238]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_SparkFun_LeftRear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear left GPS relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [15.7, 4.1, 3.238];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,4), round(expected_transformed_ENUPoint_in_dashCoord,4)));

%% Left GPS - Case 18

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into left GPS coordinates.


% A point in ENU coordinates. 
sensorReading_ENU = [-2, 9, 4];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [3,4,-3,180,270,90];

% The vehicle is moved 3 units in the X direction, 4 units in the 
% Y direction, and -3 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 180, 270, and 90, respectively
% relative to ENU coordinate
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in left GPS coordinates will be [4.9, -4.3, 4.238]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_SparkFun_LeftRear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear left GPS relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [4.9, -4.3, 4.238];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,4), round(expected_transformed_ENUPoint_in_dashCoord,4)));

%% The following cases were written to test the accuracy of right GPS coordinates 

% when a point(sensorReading) in ENU coordinates is transformed to 
% right GPS coordinates


%% right GPS - Case 1

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into right GPS coordinates.

% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,0,0,0,0,0];

% The vehicle's position is at the origin and the orientation of the
% vehicle is also zero which implies -- roll, pitch and yaw are zero
%
% Since we know the offset position of the right GPS from the sensor 
% platform and how it is oriented, the expected answer of the transformed
% sensorReading_ENU in right GPS coordinates will be [0.7, 0.762, -2.1]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_SparkFun_RightRear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear right GPS relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [0.7, 0.762, -2.1];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,4), round(expected_transformed_ENUPoint_in_dashCoord,4)));

%% right GPS - Case 2

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into right GPS coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [5,0,0,0,0,0];

% The vehicle is moved 5 units in the X - direction relative to ENU
% coordinates.
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in right GPS coordinates will be [-4.3, 0.762, -2.1]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_SparkFun_RightRearwe234';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear right GPS relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [-4.3, 0.762, -2.1];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,4), round(expected_transformed_ENUPoint_in_dashCoord,4)));

%% right GPS - Case 3

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into right GPS coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,-4,0,0,0,0];

% The vehicle is moved -4 units in the Y - direction relative to ENU
% coordinates.
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in right GPS coordinates will be [0.7, 4.762, -2.1]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_SparkFun_RightRear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear right GPS relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [0.7, 4.762, -2.1];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,4), round(expected_transformed_ENUPoint_in_dashCoord,4)));

%% right GPS - Case 4

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into right GPS coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,0,2,0,0,0];

% The vehicle is moved 2 units in the Z - direction relative to ENU
% coordinates.
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in right GPS coordinates will be [0.7, -0.762, -4.1]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_SparkFun_RightRear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear right GPS relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [0.7, 0.762, -4.1];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,4), round(expected_transformed_ENUPoint_in_dashCoord,4)));

%% right GPS - Case 5

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into right GPS coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [-8,1,-6,0,0,0];

% The vehicle is moved -8 units in the X direction, 1 units in the 
% Y direction, and -6 units in the Z direction relative to ENU coordinates.
% coordinates.
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in right GPS coordinates will be [8.7, -0.238, 3.9]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_SparkFun_RightRear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear right GPS relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [8.7, -0.238, 3.9];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,4), round(expected_transformed_ENUPoint_in_dashCoord,4)));

%% right GPS - Case 6

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into right GPS coordinates.


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
% sensorReading_ENU in right GPS coordinates will be [9.762, 4.3, -3.1]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_SparkFun_RightRear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear right GPS relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [9.762, 4.3, -3.1];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,4), round(expected_transformed_ENUPoint_in_dashCoord,4)));

%% right GPS - Case 7

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into right GPS coordinates.

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
% sensorReading_ENU in left GPS coordinates will be [-1.9, -6.238, -0.3]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_SparkFun_RightRear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear right GPS relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [-1.9, -6.238, -0.3];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,4), round(expected_transformed_ENUPoint_in_dashCoord,4)));

%% right GPS - Case 8

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into right GPS coordinates.

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
% sensorReading_ENU in left GPS coordinates will be [9.7, 5.1, 7.762]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_SparkFun_RightRear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear right GPS relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [9.7, 5.1, 7.762];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,4), round(expected_transformed_ENUPoint_in_dashCoord,4)));

%% right GPS - Case 9

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into right GPS coordinates.

% A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [3,4,-3,180,270,90];

% The vehicle is moved 3 units in the X direction, 4 units in the 
% Y direction, and -3 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 180, 270, and 90, respectively
% relative to ENU coordinate
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in left GPS coordinates will be [0.9, -2.3, -3.238]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_SparkFun_RightRear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear right GPS relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [0.9, -2.3, -3.238];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,4), round(expected_transformed_ENUPoint_in_dashCoord,4)));

%% right GPS - Case 10

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into right GPS coordinates.

% A point in ENU coordinates. 
sensorReading_ENU = [6, -4, 9];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,0,0,0,0,0];

% The vehicle's position is at the origin and the orientation of the
% vehicle is also zero which implies -- roll, pitch and yaw are zero
%
% Since we know the offset position of the right GPS from the sensor 
% platform and how it is oriented, the expected answer of the transformed
% sensorReading_ENU in right GPS coordinates will be [6.7, -4.762, 6.9]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_SparkFun_RightRear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear right GPS relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [6.7, -3.238, 6.9];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,4), round(expected_transformed_ENUPoint_in_dashCoord,4)));

%% right GPS - Case 11

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into right GPS coordinates.


% A point in ENU coordinates. 
sensorReading_ENU = [12, 4, -1];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [5,0,0,0,0,0];

% The vehicle is moved 5 units in the X - direction relative to ENU
% coordinates.
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in right GPS coordinates will be [7.7, 4.762, -3.1]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_SparkFun_RightRear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear right GPS relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [7.7, 4.762, -3.1];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,4), round(expected_transformed_ENUPoint_in_dashCoord,4)));

%% right GPS - Case 12

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into right GPS coordinates.


% A point in ENU coordinates. 
sensorReading_ENU = [-8, 9, 3];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,-4,0,0,0,0];

% The vehicle is moved -4 units in the Y - direction relative to ENU
% coordinates.
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in right GPS coordinates will be [-7.3, 13.762, 0.9]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_SparkFun_RightRear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear right GPS relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [-7.3, 13.762, 0.9];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,4), round(expected_transformed_ENUPoint_in_dashCoord,4)));

%% right GPS - Case 13

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into right GPS coordinates.


% A point in ENU coordinates. 
sensorReading_ENU = [6, 2, 9];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,0,2,0,0,0];

% The vehicle is moved 2 units in the Z - direction relative to ENU
% coordinates.
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in right GPS coordinates will be [6.7, 2.762, 4.9]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_SparkFun_RightRear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear right GPS relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [6.7, 2.762, 4.9];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,4), round(expected_transformed_ENUPoint_in_dashCoord,4)));

%% right GPS - Case 14

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into right GPS coordinates.


% A point in ENU coordinates. 
sensorReading_ENU = [11, -5, 4];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [-8,1,-6,0,0,0];

% The vehicle is moved -8 units in the X direction, 1 units in the 
% Y direction, and -6 units in the Z direction relative to ENU coordinates.
% coordinates.
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in right GPS coordinates will be [19.7, -5.238, 7.9]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_SparkFun_RightRear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear right GPS relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [19.7, -5.238, 7.9];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,4), round(expected_transformed_ENUPoint_in_dashCoord,4)));

%% right GPS - Case 15

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into right GPS coordinates.


% A point in ENU coordinates. 
sensorReading_ENU = [5, -1, 9];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [5,-9,1,0,0,90];

% The vehicle is moved 5 units in the X direction, -9 units in the 
% Y direction, and 1 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 0, 0, and 90, respectively
% relative to ENU coordinates. 
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in right GPS coordinates will be [8.762, -0.7, 5.9]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'righGPS_SparkFun_RightReartgps';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear right GPS relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [8.762, -0.7, 5.9];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,4), round(expected_transformed_ENUPoint_in_dashCoord,4)));

%% right GPS - Case 16

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into right GPS coordinates.

% A point in ENU coordinates. 
sensorReading_ENU = [-9, 2, 8];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [1,7,-4,0,90,0];

% The vehicle is moved 1 units in the X direction, 7 units in the 
% Y direction, and -4 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 0, 90, and 0, respectively
% relative to ENU coordinates.
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in left GPS coordinates will be [-9.9, -4.238, -9.3]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_SparkFun_RightRear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear right GPS relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [-9.9, -4.238, -9.3];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,4), round(expected_transformed_ENUPoint_in_dashCoord,4)));

%% right GPS - Case 17

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into right GPS coordinates.

% A point in ENU coordinates. 
sensorReading_ENU = [6, -3, 1];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [-9,-7,3,90,0,0];

% The vehicle is moved -9 units in the X direction, -7 units in the 
% Y direction, and 3 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 90, 0, and 0, respectively
% relative to ENU coordinates. 
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in left GPS coordinates will be [15.7, 4.1, 4.762]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_SparkFun_RightRearrightgps';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear right GPS relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [15.7, 4.1, 4.762];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,4), round(expected_transformed_ENUPoint_in_dashCoord,4)));

%% right GPS - Case 18

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into right GPS coordinates.

% A point in ENU coordinates. 
sensorReading_ENU = [-2, 9, 4];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [3,4,-3,180,270,90];

% The vehicle is moved 3 units in the X direction, 4 units in the 
% Y direction, and -3 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 180, 270, and 90, respectively
% relative to ENU coordinate
%
% Therefore, the expected answer of the transformed
% sensorReading_ENU in left GPS coordinates will be [4.9, -4.3, 5.762]

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'rightgpsGPS_SparkFun_RightRear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear right GPS relative to sensor platform. 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(vehicleParameters, sensorPoseParameters, in_dashCoord, vehiclePose_ENU, sensorReading_ENU, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_in_dashCoord = [4.9, -4.3, 5.762];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,4), round(expected_transformed_ENUPoint_in_dashCoord,4)));

%% Fail Conditions 

% input cannot be NaN

if 1 == 0

 % A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [NaN, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [3,4,-3,NaN,270,90];

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_SparkFun_RightRear';  

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(sensorReading_ENU, vehiclePose_ENU, in_dashCoord);
expected_transformed_ENUPoint_in_dashCoord = [0.9, -2.3, -3.238];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,4), round(expected_transformed_ENUPoint_in_dashCoord,4)));

end

%% 

% input cannot be inf

if 1 == 0

 % A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,4,-3,0,270,inf];

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_SparkFun_RightRear';  

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(sensorReading_ENU, vehiclePose_ENU, in_dashCoord);
expected_transformed_ENUPoint_in_dashCoord = [0.9, -2.3, -3.238];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,4), round(expected_transformed_ENUPoint_in_dashCoord,4)));

end

%% 

% input cannot be inf or NaN

if 1 == 0

 % A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, NaN, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [inf,4,-3,0,NaN,0];

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_SparkFun_RightRear';  

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(sensorReading_ENU, vehiclePose_ENU, in_dashCoord);
expected_transformed_ENUPoint_in_dashCoord = [0.9, -2.3, -3.238];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,4), round(expected_transformed_ENUPoint_in_dashCoord,4)));

end

%% 

% sensorReading_ENU must be 1 x 3

if 1 == 0

 % A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [3,4,-3,180,270,90];


% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_SparkFun_RightRear';  

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(sensorReading_ENU, vehiclePose_ENU, in_dashCoord);
expected_transformed_ENUPoint_in_dashCoord = [0.9, -2.3, -3.238];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,4), round(expected_transformed_ENUPoint_in_dashCoord,4)));

end

%% 

% sensorReading_ENU must be 1 x 6

if 1 == 0

 % A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [3,4,-3,180,270];


% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'rightgGPS_SparkFun_RightRearps';  

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(sensorReading_ENU, vehiclePose_ENU, in_dashCoord);
expected_transformed_ENUPoint_in_dashCoord = [0.9, -2.3, -3.238];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,4), round(expected_transformed_ENUPoint_in_dashCoord,4)));

end

%% 

% input arguments should not exceed 3

if 1 == 0

 % A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [3,4,-3,180,270];

SensorVals = [3, 4, 6];

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_SparkFun_RightRear';  

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(sensorReading_ENU, vehiclePose_ENU, in_dashCoord, SensorVals);
expected_transformed_ENUPoint_in_dashCoord = [0.9, -2.3, -3.238];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,4), round(expected_transformed_ENUPoint_in_dashCoord,4)));

end

%% 

% input arguments should not be less than 3

if 1 == 0

 % A point in ENU coordinates. In this case, the point is in origin
sensorReading_ENU = [0, 0, 0];

% Transforms sensorReading_ENU into this coordinates
in_dashCoord = 'GPS_SparkFun_RightRear';  

transformed_ENUPoint_in_dashCoord = fcn_Transform_ENUToSensorCoord(sensorReading_ENU, in_dashCoord);
expected_transformed_ENUPoint_in_dashCoord = [0.9, -2.3, -3.238];

assert(isequal(round(transformed_ENUPoint_in_dashCoord,4), round(expected_transformed_ENUPoint_in_dashCoord,4)));

end