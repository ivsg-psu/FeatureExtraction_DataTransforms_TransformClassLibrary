% script_test_fcn_Transform_SensorCoordToENU_vehicle.m
% tests fcn_Transform_SensorCoordToENU.m
%
% This script uses fcn_Transform_SensorCoordToENU.m to transform the ENU
% coordinates to vehicle coordinates. 
%
% Revision history
% 2023_09_27 - Aneesh Batchu
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

%% Load the example data

run('Example_vehicleParameters_and_sensorPoseParameters_Struct.m')

%% Vehicle Coordinates - Case 1

% In this case, we are transforming the sensorReading_SensorCoord 
% (vehicle coordinates) into ENU coordinates.


% A point in vehicle coordinates. 
sensorReading_SensorCoord = [-5, 0, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [5,0,0,0,0,0]; 

% The vehicle is moved 5 units in the X - direction relative to ENU
% coordinates. 
% 
% In "fcn_Transform_ENUToSensorCoord" cases, the sensorReading_ENU was [0,
% 0, 0]. The transformed point was [-5, 0, 0]. 
% In this case, if the sensorReading_SensorCoord = [-5, 0, 0] the expected
% answer of the transformed ENU point from vehicle coordinates will be
% [0, 0, 0]. 

% Note: The sensorReading_ENU from the "fcn_Transform_ENUToSensorCoord"
% cases is answer(expected) expected_transformed_ENUPoint_from_dashCoord
% for "fcn_Transform_SensorCoordToENU" cases.
%                         and
% The sensorReading_SensorCoord (input) for "fcn_Transform_SensorCoordToENU" 
% cases is the answer (transformed_ENUPoint_in_dashCoord) of 
% "fcn_Transform_ENUToSensorCoord" cases. 

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'vehicleCoord';  

% Perturbation in sensor Pose 
% We are transforming the point to vehicle coordinates. We cannot add
% perturbation to it
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];
 
assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 2

% In this case, we are transforming the sensorReading_SensorCoord 
% (vehicle coordinates) into ENU coordinates.


% A point in vehicle coordinates. 
sensorReading_SensorCoord = [0, 4, 0];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,-4,0,0,0,0];

% The vehicle is moved -4 units in the Y - direction relative to ENU
% coordinates. 
%
% Note: The sensorReading_ENU from the "fcn_Transform_ENUToSensorCoord"
% cases is answer(expected) expected_transformed_ENUPoint_from_dashCoord
% for "fcn_Transform_SensorCoordToENU" cases.
%                         and
% The sensorReading_SensorCoord (input) for "fcn_Transform_SensorCoordToENU" 
% cases is the answer (transformed_ENUPoint_in_dashCoord) of 
% "fcn_Transform_ENUToSensorCoord" cases. 

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'vehicle';  

% Perturbation in sensor Pose 
% We are transforming the point to vehicle coordinates. We cannot add
% perturbation to it
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];
 
assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 3

% In this case, we are transforming the sensorReading_SensorCoord 
% (vehicle coordinates) into ENU coordinates.


% A point in vehicle coordinates. 
sensorReading_SensorCoord = [0, 0, -2];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,0,2,0,0,0];

% The vehicle is moved 2 units in the Z - direction relative to ENU
% coordinates. 
%
% Note: The sensorReading_ENU from the "fcn_Transform_ENUToSensorCoord"
% cases is answer(expected) expected_transformed_ENUPoint_from_dashCoord
% for "fcn_Transform_SensorCoordToENU" cases.
%                         and
% The sensorReading_SensorCoord (input) for "fcn_Transform_SensorCoordToENU" 
% cases is the answer (transformed_ENUPoint_in_dashCoord) of 
% "fcn_Transform_ENUToSensorCoord" cases. 

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'vehicle_coord';  

% Perturbation in sensor Pose 
% We are transforming the point to vehicle coordinates. We cannot add
% perturbation to it
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];
 
assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 4 - vehilclePose_ENU with roll, pitch and yaw = 0.

% In this case, we are transforming the sensorReading_SensorCoord 
% (vehicle coordinates) into ENU coordinates.


% A point in vehicle coordinates. 
sensorReading_SensorCoord = [5, -7, -3];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [-5,7,3,0,0,0];

% The vehicle is moved -5 units in the X direction, 7 units in the 
% Y direction, and 3 units in the Z direction relative to ENU coordinates.
% coordinates. 
%
% Note: The sensorReading_ENU from the "fcn_Transform_ENUToSensorCoord"
% cases is answer(expected) expected_transformed_ENUPoint_from_dashCoord
% for "fcn_Transform_SensorCoordToENU" cases.
%                         and
% The sensorReading_SensorCoord (input) for "fcn_Transform_SensorCoordToENU" 
% cases is the answer (transformed_ENUPoint_in_dashCoord) of 
% "fcn_Transform_ENUToSensorCoord" cases. 


% Transforms this coordinates into ENU coordinates
from_dashCoord = 'vehicle';  

% Perturbation in sensor Pose 
% We are transforming the point to vehicle coordinates. We cannot add
% perturbation to it
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];
 
assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 5 - vehilclePose_ENU with yaw

% In this case, we are transforming the sensorReading_SensorCoord 
% (vehicle coordinates) into ENU coordinates.


% A point in vehicle coordinates. 
sensorReading_SensorCoord = [-5, 4, 6];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [4,5,-6,0,0,90];

% The vehicle is moved 4 units in the X direction, 5 units in the 
% Y direction, and -6 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 0, 0, and 90, respectively
% relative to ENU coordinates. 
%
% Note: The sensorReading_ENU from the "fcn_Transform_ENUToSensorCoord"
% cases is answer(expected) expected_transformed_ENUPoint_from_dashCoord
% for "fcn_Transform_SensorCoordToENU" cases.
%                         and
% The sensorReading_SensorCoord (input) for "fcn_Transform_SensorCoordToENU" 
% cases is the answer (transformed_ENUPoint_in_dashCoord) of 
% "fcn_Transform_ENUToSensorCoord" cases. 

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'vehicle';  

% Perturbation in sensor Pose 
% We are transforming the point to vehicle coordinates. We cannot add
% perturbation to it
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 6 - vehilclePose_ENU with pitch

% In this case, we are transforming the sensorReading_SensorCoord 
% (vehicle coordinates) into ENU coordinates.


% A point in vehicle coordinates. 
sensorReading_SensorCoord = [-6, -5, -4];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [4,5,-6,0,90,0];

% The vehicle is moved 4 units in the X direction, 5 units in the 
% Y direction, and -6 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 0, 90, and 0, respectively
% relative to ENU coordinates. 
%
% Note: The sensorReading_ENU from the "fcn_Transform_ENUToSensorCoord"
% cases is answer(expected) expected_transformed_ENUPoint_from_dashCoord
% for "fcn_Transform_SensorCoordToENU" cases.
%                         and
% The sensorReading_SensorCoord (input) for "fcn_Transform_SensorCoordToENU" 
% cases is the answer (transformed_ENUPoint_in_dashCoord) of 
% "fcn_Transform_ENUToSensorCoord" cases. 

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'vehicle';  

% Perturbation in sensor Pose 
% We are transforming the point to vehicle coordinates. We cannot add
% perturbation to it
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 7 - vehilclePose_ENU with position and roll 

% In this case, we are transforming the sensorReading_SensorCoord 
% (vehicle coordinates) into ENU coordinates.


% A point in vehicle coordinates. 
sensorReading_SensorCoord = [-4, -6, -5];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [4,5,-6,-90,0,0];

% The vehicle is moved 4 units in the X direction, 5 units in the 
% Y direction, and -6 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 90, 0, and 0, respectively
% relative to ENU coordinates. 
%
% Note: The sensorReading_ENU from the "fcn_Transform_ENUToSensorCoord"
% cases is answer(expected) expected_transformed_ENUPoint_from_dashCoord
% for "fcn_Transform_SensorCoordToENU" cases.
%                         and
% The sensorReading_SensorCoord (input) for "fcn_Transform_SensorCoordToENU" 
% cases is the answer (transformed_ENUPoint_in_dashCoord) of 
% "fcn_Transform_ENUToSensorCoord" cases. 

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'vehicle';  

% Perturbation in sensor Pose 
% We are transforming the point to vehicle coordinates. We cannot add
% perturbation to it
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 8 - vehilclePose_ENU with position, pitch and yaw

% In this case, we are transforming the sensorReading_SensorCoord 
% (vehicle coordinates) into ENU coordinates.


% A point in vehicle coordinates. 
sensorReading_SensorCoord = [5, -9, 1];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [1,-9,5,0,90,180];

% The vehicle is moved 1 units in the X direction, -9 units in the 
% Y direction, and 5 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 0, 90, and 180, respectively
% relative to ENU coordinates. 
%
% Note: The sensorReading_ENU from the "fcn_Transform_ENUToSensorCoord"
% cases is answer(expected) expected_transformed_ENUPoint_from_dashCoord
% for "fcn_Transform_SensorCoordToENU" cases.
%                         and
% The sensorReading_SensorCoord (input) for "fcn_Transform_SensorCoordToENU" 
% cases is the answer (transformed_ENUPoint_in_dashCoord) of 
% "fcn_Transform_ENUToSensorCoord" cases. 

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'vehicle';  

% Perturbation in sensor Pose 
% We are transforming the point to vehicle coordinates. We cannot add
% perturbation to it
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 9 - vehilclePose_ENU with position, roll and pitch

% In this case, we are transforming the sensorReading_SensorCoord 
% (vehicle coordinates) into ENU coordinates.


% A point in vehicle coordinates. 
sensorReading_SensorCoord = [1, -5, 9];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [1,-9,5,-90,180,0];

% The vehicle is moved 1 units in the X direction, -9 units in the 
% Y direction, and 5 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 0, 90, and 180, respectively
% relative to ENU coordinates. 
%
% Note: The sensorReading_ENU from the "fcn_Transform_ENUToSensorCoord"
% cases is answer(expected) expected_transformed_ENUPoint_from_dashCoord
% for "fcn_Transform_SensorCoordToENU" cases.
%                         and
% The sensorReading_SensorCoord (input) for "fcn_Transform_SensorCoordToENU" 
% cases is the answer (transformed_ENUPoint_in_dashCoord) of 
% "fcn_Transform_ENUToSensorCoord" cases. 

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'vehicle_coordinates';  

% Perturbation in sensor Pose 
% We are transforming the point to vehicle coordinates. We cannot add
% perturbation to it
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 10 - vehilclePose_ENU with position, roll and yaw

% In this case, we are transforming the sensorReading_SensorCoord 
% (vehicle coordinates) into ENU coordinates.


% A point in vehicle coordinates. 
sensorReading_SensorCoord = [9, -1, 5];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [1,-9,5,-180,0,90];

% The vehicle is moved 1 units in the X direction, -9 units in the 
% Y direction, and 5 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 180, 0, and 90, respectively
% relative to ENU coordinates. 
%
% Note: The sensorReading_ENU from the "fcn_Transform_ENUToSensorCoord"
% cases is answer(expected) expected_transformed_ENUPoint_from_dashCoord
% for "fcn_Transform_SensorCoordToENU" cases.
%                         and
% The sensorReading_SensorCoord (input) for "fcn_Transform_SensorCoordToENU" 
% cases is the answer (transformed_ENUPoint_in_dashCoord) of 
% "fcn_Transform_ENUToSensorCoord" cases. 

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'vehicle_coordinates';  

% Perturbation in sensor Pose 
% We are transforming the point to vehicle coordinates. We cannot add
% perturbation to it
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 11 - vehilclePose_ENU with position, roll, pitch and yaw

% In this case, we are transforming the sensorReading_SensorCoord 
% (vehicle coordinates) into ENU coordinates.

% A point in vehicle coordinates. 
sensorReading_SensorCoord = [5, 1, 9];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [1,-9,5,-270,90,180];

% The vehicle is moved 1 units in the X direction, -9 units in the 
% Y direction, and 5 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 270, 90, and 180, respectively
% relative to ENU coordinates. 
%
% Note: The sensorReading_ENU from the "fcn_Transform_ENUToSensorCoord"
% cases is answer(expected) expected_transformed_ENUPoint_from_dashCoord
% for "fcn_Transform_SensorCoordToENU" cases.
%                         and
% The sensorReading_SensorCoord (input) for "fcn_Transform_SensorCoordToENU" 
% cases is the answer (transformed_ENUPoint_in_dashCoord) of 
% "fcn_Transform_ENUToSensorCoord" cases. 

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'vehicle_coordinates';  

% Perturbation in sensor Pose 
% We are transforming the point to vehicle coordinates. We cannot add
% perturbation to it
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 12 - vehilclePose_ENU with position, and roll, pitch and yaw greater than 360 degrees

% In this case, we are transforming the sensorReading_SensorCoord 
% (vehicle coordinates) into ENU coordinates.


% A point in vehicle coordinates. 
sensorReading_SensorCoord = [5, 1, 9];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [1,-9,5,-(360+270),720+90,1080+180];

% The vehicle is moved 1 units in the X direction, -9 units in the 
% Y direction, and 5 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 360+270, 720+90, and 1080+180, respectively
% relative to ENU coordinates. 
%
% Note: The sensorReading_ENU from the "fcn_Transform_ENUToSensorCoord"
% cases is answer(expected) expected_transformed_ENUPoint_from_dashCoord
% for "fcn_Transform_SensorCoordToENU" cases.
%                         and
% The sensorReading_SensorCoord (input) for "fcn_Transform_SensorCoordToENU" 
% cases is the answer (transformed_ENUPoint_in_dashCoord) of 
% "fcn_Transform_ENUToSensorCoord" cases. 

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'vehicle_coordinates';  

% Perturbation in sensor Pose 
% We are transforming the point to vehicle coordinates. We cannot add
% perturbation to it
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,4), round(expected_transformed_ENUPoint_from_dashCoord,4)));

%% Vehicle Coordinates - Case 13 - vehilclePose_ENU with position, and negative roll, pitch and yaw

% In this case, we are transforming the sensorReading_SensorCoord 
% (vehicle coordinates) into ENU coordinates.


% A point in vehicle coordinates.  
sensorReading_SensorCoord = [-5, 1, -9];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [1,-9,5,270,-90,-180];

% The vehicle is moved 1 units in the X direction, -9 units in the 
% Y direction, and 5 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 270, 90, and 180, respectively
% relative to ENU coordinates. 
%
% Note: The sensorReading_ENU from the "fcn_Transform_ENUToSensorCoord"
% cases is answer(expected) expected_transformed_ENUPoint_from_dashCoord
% for "fcn_Transform_SensorCoordToENU" cases.
%                         and
% The sensorReading_SensorCoord (input) for "fcn_Transform_SensorCoordToENU" 
% cases is the answer (transformed_ENUPoint_in_dashCoord) of 
% "fcn_Transform_ENUToSensorCoord" cases. 

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'vehicle_coordinates';  

% Perturbation in sensor Pose 
% We are transforming the point to vehicle coordinates. We cannot add
% perturbation to it
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 14 - When sensorReading_ENU is not [0, 0, 0].

% In this case, we are transforming the sensorReading_SensorCoord 
% (vehicle coordinates) into ENU coordinates.


% A point in vehicle coordinates. 
sensorReading_SensorCoord = [0, -3, 2];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [5,0,0,0,0,0]; 

% The vehicle is moved 5 units in the X - direction relative to ENU
% coordinates. 
%
% Note: The sensorReading_ENU from the "fcn_Transform_ENUToSensorCoord"
% cases is answer(expected) expected_transformed_ENUPoint_from_dashCoord
% for "fcn_Transform_SensorCoordToENU" cases.
%                         and
% The sensorReading_SensorCoord (input) for "fcn_Transform_SensorCoordToENU" 
% cases is the answer (transformed_ENUPoint_in_dashCoord) of 
% "fcn_Transform_ENUToSensorCoord" cases. 

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'vehicleCoord';  

% Perturbation in sensor Pose 
% We are transforming the point to vehicle coordinates. We cannot add
% perturbation to it
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [5, -3, 2];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 15 - When sensorReading_ENU is not [0, 0, 0].

% In this case, we are transforming the sensorReading_SensorCoord 
% (vehicle coordinates) into ENU coordinates.


% A point in vehicle coordinates. 
sensorReading_SensorCoord = [-7, -10,-5];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,8,0,0,0,0]; 

% The vehicle is moved 8 units in the Y - direction relative to ENU
% coordinates. 
%
% Note: The sensorReading_ENU from the "fcn_Transform_ENUToSensorCoord"
% cases is answer(expected) expected_transformed_ENUPoint_from_dashCoord
% for "fcn_Transform_SensorCoordToENU" cases.
%                         and
% The sensorReading_SensorCoord (input) for "fcn_Transform_SensorCoordToENU" 
% cases is the answer (transformed_ENUPoint_in_dashCoord) of 
% "fcn_Transform_ENUToSensorCoord" cases. 

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'vehicle Coord';  

% Perturbation in sensor Pose 
% We are transforming the point to vehicle coordinates. We cannot add
% perturbation to it
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [-7, -2, -5];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 16 - When sensorReading_ENU is not [0, 0, 0].

% In this case, we are transforming the sensorReading_SensorCoord 
% (vehicle coordinates) into ENU coordinates.


% A point in vehicle coordinates. 
sensorReading_SensorCoord = [4, 6, 2];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,0,-6,0,0,0]; 

% The vehicle is moved -6 units in the Z - direction relative to ENU
% coordinates. 
%
% Note: The sensorReading_ENU from the "fcn_Transform_ENUToSensorCoord"
% cases is answer(expected) expected_transformed_ENUPoint_from_dashCoord
% for "fcn_Transform_SensorCoordToENU" cases.
%                         and
% The sensorReading_SensorCoord (input) for "fcn_Transform_SensorCoordToENU" 
% cases is the answer (transformed_ENUPoint_in_dashCoord) of 
% "fcn_Transform_ENUToSensorCoord" cases. 

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'vehicle Coord';  

% Perturbation in sensor Pose 
% We are transforming the point to vehicle coordinates. We cannot add
% perturbation to it
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [4, 6, -4];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 17 - When sensorReading_ENU is not [0, 0, 0].

% In this case, we are transforming the sensorReading_SensorCoord 
% (vehicle coordinates) into ENU coordinates.


% A point in vehicle coordinates. 
sensorReading_SensorCoord = [6, -8, 7];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [-4,7,2,0,0,0]; 

% The vehicle is moved -4 units in the X direction, 7 units in the 
% Y direction, and 2 units in the Z direction relative to ENU coordinates.
% coordinates. 
%
% Note: The sensorReading_ENU from the "fcn_Transform_ENUToSensorCoord"
% cases is answer(expected) expected_transformed_ENUPoint_from_dashCoord
% for "fcn_Transform_SensorCoordToENU" cases.
%                         and
% The sensorReading_SensorCoord (input) for "fcn_Transform_SensorCoordToENU" 
% cases is the answer (transformed_ENUPoint_in_dashCoord) of 
% "fcn_Transform_ENUToSensorCoord" cases. 

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'vehicle Coord';  

% Perturbation in sensor Pose 
% We are transforming the point to vehicle coordinates. We cannot add
% perturbation to it
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [2, -1, 9];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 18 - When sensorReading_ENU is not [0, 0, 0].

% In this case, we are transforming the sensorReading_SensorCoord 
% (vehicle coordinates) into ENU coordinates.


% A point in vehicle coordinates. 
sensorReading_SensorCoord = [-8, -6, 7];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [-4,7,2,0,0,90]; 

% The vehicle is moved -4 units in the X direction, 7 units in the 
% Y direction, and 2 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 0, 0, and 90, respectively
% relative to ENU coordinates. 
%
% Note: The sensorReading_ENU from the "fcn_Transform_ENUToSensorCoord"
% cases is answer(expected) expected_transformed_ENUPoint_from_dashCoord
% for "fcn_Transform_SensorCoordToENU" cases.
%                         and
% The sensorReading_SensorCoord (input) for "fcn_Transform_SensorCoordToENU" 
% cases is the answer (transformed_ENUPoint_in_dashCoord) of 
% "fcn_Transform_ENUToSensorCoord" cases. 

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'vehicle Coord';  

% Perturbation in sensor Pose 
% We are transforming the point to vehicle coordinates. We cannot add
% perturbation to it
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [2, -1, 9];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 19 - When sensorReading_ENU is not [0, 0, 0].

% In this case, we are transforming the sensorReading_SensorCoord 
% (vehicle coordinates) into ENU coordinates.

% A point in vehicle coordinates. 
sensorReading_SensorCoord = [-7, -8, 6];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [-4,7,2,0,90,0]; 

% The vehicle is moved -4 units in the X direction, 7 units in the 
% Y direction, and 2 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 0, 90, and 0, respectively
% relative to ENU coordinates. 
%
% Note: The sensorReading_ENU from the "fcn_Transform_ENUToSensorCoord"
% cases is answer(expected) expected_transformed_ENUPoint_from_dashCoord
% for "fcn_Transform_SensorCoordToENU" cases.
%                         and
% The sensorReading_SensorCoord (input) for "fcn_Transform_SensorCoordToENU" 
% cases is the answer (transformed_ENUPoint_in_dashCoord) of 
% "fcn_Transform_ENUToSensorCoord" cases. 

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'vehicle Coord';  

% Perturbation in sensor Pose 
% We are transforming the point to vehicle coordinates. We cannot add
% perturbation to it
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [2, -1, 9];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 20 - When sensorReading_ENU is not [0, 0, 0].

% In this case, we are transforming the sensorReading_SensorCoord 
% (vehicle coordinates) into ENU coordinates.


% A point in vehicle coordinates. 
sensorReading_SensorCoord = [6, -7, -8];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [-4,7,2,-90,0,0]; 

% The vehicle is moved -4 units in the X direction, 7 units in the 
% Y direction, and 2 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 90, 0, and 0, respectively
% relative to ENU coordinates. 
%
% Note: The sensorReading_ENU from the "fcn_Transform_ENUToSensorCoord"
% cases is answer(expected) expected_transformed_ENUPoint_from_dashCoord
% for "fcn_Transform_SensorCoordToENU" cases.
%                         and
% The sensorReading_SensorCoord (input) for "fcn_Transform_SensorCoordToENU" 
% cases is the answer (transformed_ENUPoint_in_dashCoord) of 
% "fcn_Transform_ENUToSensorCoord" cases. 

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'vehicle Coord';  

% Perturbation in sensor Pose 
% We are transforming the point to vehicle coordinates. We cannot add
% perturbation to it
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [2, -1, 9];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Vehicle Coordinates - Case 21 - When sensorReading_ENU is not [0, 0, 0].

% In this case, we are transforming the sensorReading_SensorCoord 
% (vehicle coordinates) into ENU coordinates.


% A point in vehicle coordinates. 
sensorReading_SensorCoord = [-8, 7, 6];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [-4,7,2,-90,180,270]; 

% The vehicle is moved -4 units in the X direction, 7 units in the 
% Y direction, and 2 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 90, 180, and 270, respectively
% relative to ENU coordinates. 
%
% Note: The sensorReading_ENU from the "fcn_Transform_ENUToSensorCoord"
% cases is answer(expected) expected_transformed_ENUPoint_from_dashCoord
% for "fcn_Transform_SensorCoordToENU" cases.
%                         and
% The sensorReading_SensorCoord (input) for "fcn_Transform_SensorCoordToENU" 
% cases is the answer (transformed_ENUPoint_in_dashCoord) of 
% "fcn_Transform_ENUToSensorCoord" cases. 

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'vehicle Coord';  

% Perturbation in sensor Pose 
% We are transforming the point to vehicle coordinates. We cannot add
% perturbation to it
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [2, -1, 9];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

