% script_test_fcn_Transform_SensorCoordToENU_GPSSensorPlatformRear.m
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

%% Sensor Platform - Case 1

% In this case, we are transforming the sensorReading_SensorCoord 
% (sensor platform coordinates) into ENU coordinates.


% A point in sensor platform coordinates. 
sensorReading_SensorCoord = [0.6027 0 -1.5261];

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
from_dashCoord = 'GPS_Hemisphere_SensorPlatform_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the sensor platform relative to vehicle body 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));


%% Sensor Platform - Case 2

% In this case, we are transforming the sensorReading_SensorCoord 
% (sensor platform coordinates) into ENU coordinates.


% A point in sensor platform coordinates. 
sensorReading_SensorCoord = [-4.3973 0 -1.5261];

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
from_dashCoord = 'GPS_Hemisphere_SensorPlatform_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the sensor platform relative to vehicle body 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,6), round(expected_transformed_ENUPoint_from_dashCoord,6)));

%% Sensor Platform - Case 3

% In this case, we are transforming the sensorReading_SensorCoord 
% (sensor platform coordinates) into ENU coordinates.


% A point in sensor platform coordinates. 
sensorReading_SensorCoord = [0.6027 4 -1.5261];

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
from_dashCoord = 'GPS_Hemisphere_SensorPlatform_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the sensor platform relative to vehicle body 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Sensor Platform - Case 4

% In this case, we are transforming the sensorReading_SensorCoord 
% (sensor platform coordinates) into ENU coordinates.


% A point in sensor platform coordinates. 
sensorReading_SensorCoord = [0.6027 0 -3.5261];

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
from_dashCoord = 'GPS_Hemisphere_SensorPlatform_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the sensor platform relative to vehicle body 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Sensor Platform - Case 5

% In this case, we are transforming the sensorReading_SensorCoord 
% (sensor platform coordinates) into ENU coordinates.


% A point in sensor platform coordinates. 
sensorReading_SensorCoord = [8.6027 -1 4.4739];

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
from_dashCoord = 'GPS_Hemisphere_SensorPlatform_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the sensor platform relative to vehicle body 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Sensor Platform - Case 6

% In this case, we are transforming the sensorReading_SensorCoord 
% (sensor platform coordinates) into ENU coordinates.


% A point in sensor platform coordinates. 
sensorReading_SensorCoord = [9.6027 5 -2.5261];

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
from_dashCoord = 'GPS_Hemisphere_SensorPlatform_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the sensor platform relative to vehicle body 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Sensor Platform - Case 7

% In this case, we are transforming the sensorReading_SensorCoord 
% (sensor platform coordinates) into ENU coordinates.


% A point in sensor platform coordinates. 
sensorReading_SensorCoord = [-3.3973 -7 -2.5261];

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
from_dashCoord = 'GPS_Hemisphere_SensorPlatform_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the sensor platform relative to vehicle body 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Sensor Platform - Case 8

% In this case, we are transforming the sensorReading_SensorCoord 
% (sensor platform coordinates) into ENU coordinates.


% A point in sensor platform coordinates. 
sensorReading_SensorCoord = [9.6027 -3 -8.5261];

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
from_dashCoord = 'GPS_Hemisphere_SensorPlatform_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the sensor platform relative to vehicle body 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Sensor Platform - Case 9

% In this case, we are transforming the sensorReading_SensorCoord 
% (sensor platform coordinates) into ENU coordinates.


% A point in sensor platform coordinates. 
sensorReading_SensorCoord = [3.6027 -3 -5.5261];

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
from_dashCoord = 'GPS_Hemisphere_SensorPlatform_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the sensor platform relative to vehicle body 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [0, 0, 0];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Sensor Platform - Case 10

% In this case, we are transforming the sensorReading_SensorCoord 
% (sensor platform coordinates) into ENU coordinates.


% A point in sensor platform coordinates. 
sensorReading_SensorCoord = [5.6027 3 -3.5261];

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
from_dashCoord = 'GPS_Hemisphere_SensorPlatform_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the sensor platform relative to vehicle body 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [5, 3, -2];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));


%% Sensor Platform - Case 11

% In this case, we are transforming the sensorReading_SensorCoord 
% (sensor platform coordinates) into ENU coordinates.


% A point in sensor platform coordinates. 
sensorReading_SensorCoord = [-3.3973 -3 4.4739];

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
from_dashCoord = 'GPS_Hemisphere_SensorPlatform_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the sensor platform relative to vehicle body 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [1, -3, 6];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,6), round(expected_transformed_ENUPoint_from_dashCoord,6)));

%% Sensor Platform - Case 12

% In this case, we are transforming the sensorReading_SensorCoord 
% (sensor platform coordinates) into ENU coordinates.


% A point in sensor platform coordinates. 
sensorReading_SensorCoord = [-3.3973 11 -2.5261];

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
from_dashCoord = 'GPS_Hemisphere_SensorPlatform_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the sensor platform relative to vehicle body 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [-4, 7, -1];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Sensor Platform - Case 13

% In this case, we are transforming the sensorReading_SensorCoord 
% (sensor platform coordinates) into ENU coordinates.


% A point in sensor platform coordinates. 
sensorReading_SensorCoord = [7.6027 2 -7.5261];

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
from_dashCoord = 'GPS_Hemisphere_SensorPlatform_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the sensor platform relative to vehicle body 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [7, 2, -4];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,6), round(expected_transformed_ENUPoint_from_dashCoord,6)));

%% Sensor Platform - Case 14

% In this case, we are transforming the sensorReading_SensorCoord 
% (sensor platform coordinates) into ENU coordinates.


% A point in sensor platform coordinates. 
sensorReading_SensorCoord = [10.6027 -8 15.4739];

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
from_dashCoord = 'GPS_Hemisphere_SensorPlatform_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the sensor platform relative to vehicle body 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [2, -7, 11];

assert(isequal(transformed_ENUPoint_from_dashCoord, expected_transformed_ENUPoint_from_dashCoord));

%% Sensor Platform - Case 15

% In this case, we are transforming the sensorReading_SensorCoord 
% (sensor platform coordinates) into ENU coordinates.

% A point in sensor platform coordinates. 
sensorReading_SensorCoord = [7.6027 4 0.4739];

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
from_dashCoord = 'GPS_Hemisphere_SensorPlatform_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the sensor platform relative to vehicle body 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [1, -2, 3];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,6), round(expected_transformed_ENUPoint_from_dashCoord,6)));

%% Sensor Platform - Case 16

% In this case, we are transforming the sensorReading_SensorCoord 
% (sensor platform coordinates) into ENU coordinates.

% A point in sensor platform coordinates. 
sensorReading_SensorCoord = [-4.3973 1 -4.5261];

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
from_dashCoord = 'GPS_Hemisphere_SensorPlatform_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the sensor platform relative to vehicle body 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [-2, 8, 1];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,6), round(expected_transformed_ENUPoint_from_dashCoord,6)));

%% Sensor Platform - Case 17

% In this case, we are transforming the sensorReading_SensorCoord 
% (sensor platform coordinates) into ENU coordinates.

% A point in sensor platform coordinates. 
sensorReading_SensorCoord = [14.6027 -1 -9.5261];

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
% sensorReading_ENU in sensor platform coordinates will be [15, 2.6, 8]

% Transforms this coordinates into ENU coordinates
from_dashCoord = 'GPS_Hemisphere_SensorPlatform_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the sensor platform relative to vehicle body 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [5, 1, 2];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,6), round(expected_transformed_ENUPoint_from_dashCoord,6)));

%% Sensor Platform - Case 18

% In this case, we are transforming the sensorReading_SensorCoord 
% (sensor platform coordinates) into ENU coordinates.

% A point in sensor platform coordinates. 
sensorReading_SensorCoord = [6.6027 -2 -4.5261];

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
from_dashCoord = 'GPS_Hemisphere_SensorPlatform_Rear';  

% Perturbation in the position (in cm) and orientation (in deg) of 
% the sensor platform relative to vehicle body 
perturbation_in_sensorPose = [];

% If you want a plot, you can give some number here. 
fig_num = [];

transformed_ENUPoint_from_dashCoord = fcn_Transform_SensorCoordToENU(vehicleParameters, sensorPoseParameters, from_dashCoord, vehiclePose_ENU, sensorReading_SensorCoord, perturbation_in_sensorPose, fig_num);
expected_transformed_ENUPoint_from_dashCoord = [1, 1, 3];

assert(isequal(round(transformed_ENUPoint_from_dashCoord,6), round(expected_transformed_ENUPoint_from_dashCoord,6)));
