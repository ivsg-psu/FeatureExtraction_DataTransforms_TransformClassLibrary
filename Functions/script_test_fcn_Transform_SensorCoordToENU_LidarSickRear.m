% script_test_fcn_Transform_SensorCoordToENU_LidarSickRear.m
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

%% Sick Lidar - Case 1

% In this case, we are transforming the sensorReading_SensorCoord 
% (sick lidar coordinates) into ENU coordinates.


% A point in sick lidar coordinates.  
sensorReading_SensorCoord = [0 1.54 0.9858];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,0,0,0,0,0];

% The vehicle's position is at the origin and the orientation of the
% vehicle is also zero which implies -- roll, pitch and yaw are zero
%
% Note: The sensorReading_ENU from the "fcn_Transform_ENUToSensorCoord"
% cases is answer(expected) expected_transformed_ENUPoint_from_dashCoord
% for "fcn_Transform_SensorCoordToENU" cases.
%                         and
% The sensorReading_SensorCoord (input) for "fcn_Transform_SensorCoordToENU" 
% cases is the answer (transformed_ENUPoint_in_dashCoord) of 
% "fcn_Transform_ENUToSensorCoord" cases.

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Sick Lidar - Case 2

% In this case, we are transforming the sensorReading_SensorCoord 
% (sick lidar coordinates) into ENU coordinates.


% A point in sick lidar coordinates.  
sensorReading_SensorCoord = [0 1.54 -4.0142];

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
from_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Sick Lidar - Case 3

% In this case, we are transforming the sensorReading_SensorCoord 
% (sick lidar coordinates) into ENU coordinates.


% A point in sick lidar coordinates.  
sensorReading_SensorCoord = [-4 1.54 0.9858];

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
from_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Sick Lidar - Case 4

% In this case, we are transforming the sensorReading_SensorCoord 
% (sick lidar coordinates) into ENU coordinates.


% A point in sick lidar coordinates.  
sensorReading_SensorCoord = [0 3.54 0.9858];

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
from_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,6), round(expected_transformed_ENUPoint_from_dashCoord,6)));

%% Sick Lidar - Case 5

% In this case, we are transforming the sensorReading_SensorCoord 
% (sick lidar coordinates) into ENU coordinates.


% A point in sick lidar coordinates.  
sensorReading_SensorCoord = [1 -4.46 8.9858];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [-8,1,-6,0,0,0];

% The vehicle is moved -8 units in the X direction, 1 units in the 
% Y direction, and -6 units in the Z direction relative to ENU coordinates.
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
from_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,6), round(expected_transformed_ENUPoint_from_dashCoord,6)));

%% Sick Lidar - Case 6

% In this case, we are transforming the sensorReading_SensorCoord 
% (sick lidar coordinates) into ENU coordinates.


% A point in sick lidar coordinates.  
sensorReading_SensorCoord = [-5 2.54 9.9858];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [5,-9,1,0,0,90];

% The vehicle is moved 5 units in the X direction, -9 units in the 
% Y direction, and 1 units in the Z direction relative to ENU coordinates.
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
from_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,6), round(expected_transformed_ENUPoint_from_dashCoord,6)));


%% Sick Lidar - Case 7

% In this case, we are transforming the sensorReading_SensorCoord 
% (sick lidar coordinates) into ENU coordinates.


% A point in sick lidar coordinates.  
sensorReading_SensorCoord = [7 2.54 -3.0142];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [1,7,-4,0,90,0];

% The vehicle is moved 1 units in the X direction, 7 units in the 
% Y direction, and -4 units in the Z direction relative to ENU coordinates.
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
from_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,6), round(expected_transformed_ENUPoint_from_dashCoord,6)));


%% Sick Lidar - Case 8

% In this case, we are transforming the sensorReading_SensorCoord 
% (sick lidar coordinates) into ENU coordinates.


% A point in sick lidar coordinates.  
sensorReading_SensorCoord = [3 8.54 9.9858];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [-9,-7,3,90,0,0];

% The vehicle is moved -9 units in the X direction, -7 units in the 
% Y direction, and 3 units in the Z direction relative to ENU coordinates.
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
from_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,6), round(expected_transformed_ENUPoint_from_dashCoord,6)));


%% Sick Lidar - Case 9

% In this case, we are transforming the sensorReading_SensorCoord 
% (sick lidar coordinates) into ENU coordinates.


% A point in sick lidar coordinates.  
sensorReading_SensorCoord = [3 5.54 3.9858];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [3,4,-3,180,270,90];

% The vehicle is moved 3 units in the X direction, 4 units in the 
% Y direction, and -3 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 180, 270, and 90, respectively
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
from_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,6), round(expected_transformed_ENUPoint_from_dashCoord,6)));

%% Sick Lidar - Case 10

% In this case, we are transforming the sensorReading_SensorCoord 
% (sick lidar coordinates) into ENU coordinates.

% A point in sick lidar coordinates. 
sensorReading_SensorCoord = [-4 6.54 3.9858];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,0,0,0,0,0];

% The vehicle's position is at the origin and the orientation of the
% vehicle is also zero which implies -- roll, pitch and yaw are zero
%
% Note: The sensorReading_ENU from the "fcn_Transform_ENUToSensorCoord"
% cases is answer(expected) expected_transformed_ENUPoint_from_dashCoord
% for "fcn_Transform_SensorCoordToENU" cases.
%                         and
% The sensorReading_SensorCoord (input) for "fcn_Transform_SensorCoordToENU" 
% cases is the answer (transformed_ENUPoint_in_dashCoord) of 
% "fcn_Transform_ENUToSensorCoord" cases.

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [3, 4, -5];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,6), round(expected_transformed_ENUPoint_from_dashCoord,6)));

%% Sick Lidar - Case 11

% In this case, we are transforming the sensorReading_SensorCoord 
% (sick lidar coordinates) into ENU coordinates.


% A point in sick lidar coordinates. 
sensorReading_SensorCoord = [6 -6.46 -3.0142];

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
from_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [1, -6, 8];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,6), round(expected_transformed_ENUPoint_from_dashCoord,6)));

%% Sick Lidar - Case 12

% In this case, we are transforming the sensorReading_SensorCoord 
% (sick lidar coordinates) into ENU coordinates.


% A point in sick lidar coordinates. 
sensorReading_SensorCoord = [-6 0.54 -3.0142];

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
from_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [-4, 2, 1];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Sick Lidar - Case 13

% In this case, we are transforming the sensorReading_SensorCoord 
% (sick lidar coordinates) into ENU coordinates.


% A point in sick lidar coordinates. 
sensorReading_SensorCoord = [-6 6.54 2.9858];

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
from_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [2, 6, -3];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,6), round(expected_transformed_ENUPoint_from_dashCoord,6)));

%% Sick Lidar - Case 14

% In this case, we are transforming the sensorReading_SensorCoord 
% (sick lidar coordinates) into ENU coordinates.


% A point in sick lidar coordinates.
sensorReading_SensorCoord = [-3 -7.46 9.9858];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [-8,1,-6,0,0,0];

% The vehicle is moved -8 units in the X direction, 1 units in the 
% Y direction, and -6 units in the Z direction relative to ENU coordinates.
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
from_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [1, 4, 3];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,6), round(expected_transformed_ENUPoint_from_dashCoord,6)));

%% Sick Lidar - Case 15

% In this case, we are transforming the sensorReading_SensorCoord 
% (sick lidar coordinates) into ENU coordinates.


% A point in sick lidar coordinates.
sensorReading_SensorCoord = [-11 -4.46 11.9858];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [5,-9,1,0,0,90];

% The vehicle is moved 5 units in the X direction, -9 units in the 
% Y direction, and 1 units in the Z direction relative to ENU coordinates.
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
from_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [-6, 2, 7];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,6), round(expected_transformed_ENUPoint_from_dashCoord,6)));

%% Sick Lidar - Case 16

% In this case, we are transforming the sensorReading_SensorCoord 
% (sick lidar coordinates) into ENU coordinates.

% A point in sick lidar coordinates. 
sensorReading_SensorCoord = [1 -0.46 -4.0142];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [1,7,-4,0,90,0];

% The vehicle is moved 1 units in the X direction, 7 units in the 
% Y direction, and -4 units in the Z direction relative to ENU coordinates.
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
from_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [3, 6, 1];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Sick Lidar - Case 17

% In this case, we are transforming the sensorReading_SensorCoord 
% (sick lidar coordinates) into ENU coordinates.


% A point in sick lidar coordinates. 
sensorReading_SensorCoord = [-5 17.54 12.9858];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [-9,-7,3,90,0,0];

% The vehicle is moved -9 units in the X direction, -7 units in the 
% Y direction, and 3 units in the Z direction relative to ENU coordinates.
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
from_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [3, 9, 8];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,6), round(expected_transformed_ENUPoint_from_dashCoord,6)));

%% Sick Lidar - Case 18

% In this case, we are transforming the sensorReading_SensorCoord 
% (sick lidar coordinates) into ENU coordinates.


% A point in sick lidar coordinates. 
sensorReading_SensorCoord = [-5 4.54 0.9858];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [3,4,-3,180,270,90];

% The vehicle is moved 3 units in the X direction, 4 units in the 
% Y direction, and -3 units in the Z direction relative to ENU coordinates.
% coordinates. The roll, pitch and yaw are 180, 270, and 90, respectively
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
from_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [8, 1, -3];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,6), round(expected_transformed_ENUPoint_from_dashCoord,6)));

%% Sick Lidar with perturbations - Case 19 

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sick lidar coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [0 1.54 -0.0142];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,0,0,0,0,0];

% The vehicle's position is at the origin and the orientation of the
% vehicle is also zero which implies -- roll, pitch and yaw are zero
%
% Since we know the offset position of the sick lidar from the sensor 
% platform and how it is oriented, the expected answer of the transformed
% sensorReading_ENU in sick lidar coordinates will be [1.5, 0, 0.4]

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform 
perturbation_in_sensorPose = [100, 0, 0, 0, 0, 0];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,6), round(expected_transformed_ENUPoint_from_dashCoord,6)));

%% Sick Lidar with perturbations - Case 20

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sick lidar coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [0.05 1.74 -0.0142];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,0,0,0,0,0];

% The vehicle's position is at the origin and the orientation of the
% vehicle is also zero which implies -- roll, pitch and yaw are zero
%
% Since we know the offset position of the sick lidar from the sensor 
% platform and how it is oriented, the expected answer of the transformed
% sensorReading_ENU in sick lidar coordinates will be [1.7, -0.05, 0.4]

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform
perturbation_in_sensorPose = [100, 5, 20, 0, 0, 0]; 

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,6), round(expected_transformed_ENUPoint_from_dashCoord,6)));

%% Sick Lidar with perturbations - Case 21

% In this case, we are transforming the sensorReading_ENU (in ENU
% coordinates) into sick lidar coordinates.


% A point in ENU coordinates. In this case, the point is in origin
sensorReading_SensorCoord = [-1.54 0 0.9858];

% vehiclePose_ENU = [x,y,z,roll,pitch,yaw]
vehiclePose_ENU = [0,0,0,0,0,0];

% The vehicle's position is at the origin and the orientation of the
% vehicle is also zero which implies -- roll, pitch and yaw are zero
%
% Since we know the offset position of the sick lidar from the sensor 
% platform and how it is oriented, the expected answer of the transformed
% sensorReading_ENU in sick lidar coordinates will be [1.4, 0, -1.5]

% Transforms sensorReading_ENU into this coordinates
from_dashCoord = 'Lidar_Sick_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the rear sick lidar relative to rear GPS hemisphere sensor platform 
perturbation_in_sensorPose = [0, 0, 0, 0, -90, 0];


% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,6), round(expected_transformed_ENUPoint_from_dashCoord,6)));